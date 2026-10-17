#include <engine/Expression.h>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace engine {

namespace {

using cd = std::complex<double>;

// Thrown internally by the tokenizer/parser; converted to the `error` out-param
// in Expression::parse so callers never see exceptions.
struct ParseError {
    std::string message;
};

[[noreturn]] void fail(const std::string& what, size_t pos)
{
    throw ParseError{what + " at character " + std::to_string(pos + 1)};
}

// ---- function + constant tables ------------------------------------------

// One row per callable function: CPU evaluation (complex and real), and the GLSL
// emit pattern, where "$" stands for the already-generated argument expression.
// `evalR` is the real-line restriction used by evalReal(); the complex-only names
// collapse to their real meaning there (conj/re = identity, im = 0, arg = 0 or pi).
struct FuncDef {
    const char* name;
    cd (*eval)(const cd&);
    double (*evalR)(double);
    const char* glslPattern;
};

const double kPi = 3.14159265358979323846;

const FuncDef kFuncs[] = {
    {"sin",  [](const cd& z) { return std::sin(z); },  [](double x) { return std::sin(x); },  "cSin($)"},
    {"cos",  [](const cd& z) { return std::cos(z); },  [](double x) { return std::cos(x); },  "cCos($)"},
    {"tan",  [](const cd& z) { return std::tan(z); },  [](double x) { return std::tan(x); },  "cTan($)"},
    {"sinh", [](const cd& z) { return std::sinh(z); }, [](double x) { return std::sinh(x); }, "cSinh($)"},
    {"cosh", [](const cd& z) { return std::cosh(z); }, [](double x) { return std::cosh(x); }, "cCosh($)"},
    {"tanh", [](const cd& z) { return std::tanh(z); }, [](double x) { return std::tanh(x); }, "cTanh($)"},
    {"exp",  [](const cd& z) { return std::exp(z); },  [](double x) { return std::exp(x); },  "cExp($)"},
    {"log",  [](const cd& z) { return std::log(z); },  [](double x) { return std::log(x); },  "cLog($)"},
    {"ln",   [](const cd& z) { return std::log(z); },  [](double x) { return std::log(x); },  "cLog($)"},
    {"sqrt", [](const cd& z) { return std::sqrt(z); }, [](double x) { return std::sqrt(x); }, "cSqrt($)"},
    {"conj", [](const cd& z) { return std::conj(z); }, [](double x) { return x; },            "cConj($)"},
    {"abs",  [](const cd& z) { return cd(std::abs(z), 0.0); }, [](double x) { return std::abs(x); },       "vec2(cMod($), 0.0)"},
    {"arg",  [](const cd& z) { return cd(std::arg(z), 0.0); }, [](double x) { return x < 0.0 ? kPi : 0.0; }, "vec2(cArg($), 0.0)"},
    {"re",   [](const cd& z) { return cd(z.real(), 0.0); },    [](double x) { return x; },                 "vec2(($).x, 0.0)"},
    {"im",   [](const cd& z) { return cd(z.imag(), 0.0); },    [](double)   { return 0.0; },               "vec2(($).y, 0.0)"},
};

const FuncDef* findFunc(const std::string& name)
{
    for (const FuncDef& f : kFuncs)
        if (name == f.name)
            return &f;
    return nullptr;
}

struct ConstDef {
    const char* name;
    cd value;
};

const ConstDef kConsts[] = {
    {"i",   {0.0, 1.0}},
    {"pi",  {3.14159265358979323846, 0.0}},
    {"tau", {6.28318530717958647692, 0.0}},
    {"e",   {2.71828182845904523536, 0.0}},
};

const ConstDef* findConst(const std::string& name)
{
    for (const ConstDef& c : kConsts)
        if (name == c.name)
            return &c;
    return nullptr;
}

// ---- tokenizer -------------------------------------------------------------

struct Token {
    enum Kind { Number, Ident, Op, End } kind;
    double number = 0.0;  // Number
    std::string text;     // Ident
    char op = 0;          // Op: one of + - * / ^ ( )
    size_t pos = 0;       // offset in the source, for error messages
};

std::vector<Token> tokenize(const std::string& src)
{
    std::vector<Token> tokens;
    size_t i = 0;
    while (i < src.size())
    {
        char c = src[i];
        if (std::isspace((unsigned char)c))
        {
            ++i;
        }
        else if (std::isdigit((unsigned char)c) || c == '.')
        {
            char* end = nullptr;
            double v = std::strtod(src.c_str() + i, &end);
            size_t len = end - (src.c_str() + i);
            if (len == 0)
                fail("malformed number", i);
            tokens.push_back({Token::Number, v, "", 0, i});
            i += len;
        }
        else if (std::isalpha((unsigned char)c) || c == '_')
        {
            size_t start = i;
            while (i < src.size() && (std::isalnum((unsigned char)src[i]) || src[i] == '_'))
                ++i;
            tokens.push_back({Token::Ident, 0.0, src.substr(start, i - start), 0, start});
        }
        else if (c == '+' || c == '-' || c == '*' || c == '/' || c == '^' || c == '(' || c == ')')
        {
            tokens.push_back({Token::Op, 0.0, "", c, i});
            ++i;
        }
        else
        {
            fail(std::string("unexpected character '") + c + "'", i);
        }
    }
    tokens.push_back({Token::End, 0.0, "", 0, src.size()});
    return tokens;
}

} // namespace

// ---- AST -------------------------------------------------------------------

struct Expression::Node {
    enum class Kind { Constant, Variable, Unary, Binary, Call };

    Kind kind;
    cd value{};                 // Constant
    int index = 0;              // Variable: variables[] index; Call: kFuncs[] index
    char op = 0;                // Binary: + - * / ^ ; Unary: -
    std::unique_ptr<Node> a;    // Unary/Call operand, Binary lhs
    std::unique_ptr<Node> b;    // Binary rhs
};

namespace {

using Node = Expression::Node;
using NodePtr = std::unique_ptr<Node>;

NodePtr makeConstant(cd value)
{
    auto n = std::make_unique<Node>();
    n->kind = Node::Kind::Constant;
    n->value = value;
    return n;
}

NodePtr makeBinary(char op, NodePtr lhs, NodePtr rhs)
{
    auto n = std::make_unique<Node>();
    n->kind = Node::Kind::Binary;
    n->op = op;
    n->a = std::move(lhs);
    n->b = std::move(rhs);
    return n;
}

// ---- recursive-descent parser ----------------------------------------------
//
// expression := term (('+'|'-') term)*
// term       := unary (('*'|'/') unary | <implicit-mult> power)*
// unary      := ('-'|'+') unary | power
// power      := primary ('^' unary)?          // right-assoc; allows z^-2
// primary    := number | constant | variable | func '(' expression ')' | '(' expression ')'

struct Parser {
    const std::vector<Token>& tokens;
    const std::vector<std::string>& variables;
    size_t cur = 0;

    const Token& peek() const { return tokens[cur]; }

    bool acceptOp(char c)
    {
        if (peek().kind == Token::Op && peek().op == c)
        {
            ++cur;
            return true;
        }
        return false;
    }

    void expectOp(char c)
    {
        if (!acceptOp(c))
            fail(std::string("expected '") + c + "'", peek().pos);
    }

    NodePtr parseExpression()
    {
        NodePtr node = parseTerm();
        for (;;)
        {
            if (acceptOp('+'))
                node = makeBinary('+', std::move(node), parseTerm());
            else if (acceptOp('-'))
                node = makeBinary('-', std::move(node), parseTerm());
            else
                return node;
        }
    }

    NodePtr parseTerm()
    {
        NodePtr node = parseUnary();
        for (;;)
        {
            if (acceptOp('*'))
                node = makeBinary('*', std::move(node), parseUnary());
            else if (acceptOp('/'))
                node = makeBinary('/', std::move(node), parseUnary());
            else if (peek().kind == Token::Number || peek().kind == Token::Ident
                     || (peek().kind == Token::Op && peek().op == '('))
                node = makeBinary('*', std::move(node), parsePower()); // implicit: 2z, (z-1)(z+1)
            else
                return node;
        }
    }

    NodePtr parseUnary()
    {
        if (acceptOp('-'))
        {
            auto n = std::make_unique<Node>();
            n->kind = Node::Kind::Unary;
            n->op = '-';
            n->a = parseUnary();
            return n;
        }
        if (acceptOp('+'))
            return parseUnary();
        return parsePower();
    }

    NodePtr parsePower()
    {
        NodePtr base = parsePrimary();
        if (acceptOp('^'))
            return makeBinary('^', std::move(base), parseUnary());
        return base;
    }

    NodePtr parsePrimary()
    {
        const Token& t = peek();
        if (t.kind == Token::Number)
        {
            ++cur;
            return makeConstant(cd(t.number, 0.0));
        }
        if (t.kind == Token::Ident)
        {
            ++cur;
            for (size_t v = 0; v < variables.size(); ++v)
            {
                if (t.text == variables[v])
                {
                    auto n = std::make_unique<Node>();
                    n->kind = Node::Kind::Variable;
                    n->index = (int)v;
                    return n;
                }
            }
            if (const ConstDef* c = findConst(t.text))
                return makeConstant(c->value);
            if (const FuncDef* f = findFunc(t.text))
            {
                expectOp('(');
                NodePtr arg = parseExpression();
                expectOp(')');
                auto n = std::make_unique<Node>();
                n->kind = Node::Kind::Call;
                n->index = (int)(f - kFuncs);
                n->a = std::move(arg);
                return n;
            }
            fail("unknown name '" + t.text + "'", t.pos);
        }
        if (acceptOp('('))
        {
            NodePtr node = parseExpression();
            expectOp(')');
            return node;
        }
        if (t.kind == Token::End)
            fail("unexpected end of expression", t.pos);
        fail(std::string("unexpected '") + t.op + "'", t.pos);
    }
};

// ---- CPU evaluation ---------------------------------------------------------

// Exponents that are small real integers use repeated multiplication instead of
// the exp/log-based pow, so z^2 is exact and finite at z = 0 (pow would NaN there,
// which ruins domain coloring). Mirrored by cPowi on the GLSL side.
bool integerExponent(const Node& exponent, long& out)
{
    if (exponent.kind != Node::Kind::Constant || exponent.value.imag() != 0.0)
        return false;
    double r = exponent.value.real();
    if (r != std::floor(r) || std::abs(r) > 32.0)
        return false;
    out = (long)r;
    return true;
}

cd powInt(cd base, long n)
{
    cd r(1.0, 0.0);
    for (long k = 0; k < std::labs(n); ++k)
        r *= base;
    return n < 0 ? cd(1.0, 0.0) / r : r;
}

cd evalNode(const Node& node, const std::vector<cd>& values)
{
    switch (node.kind)
    {
    case Node::Kind::Constant: return node.value;
    case Node::Kind::Variable: return values.at(node.index);
    case Node::Kind::Unary:    return -evalNode(*node.a, values);
    case Node::Kind::Call:     return kFuncs[node.index].eval(evalNode(*node.a, values));
    case Node::Kind::Binary:
    {
        cd lhs = evalNode(*node.a, values);
        switch (node.op)
        {
        case '+': return lhs + evalNode(*node.b, values);
        case '-': return lhs - evalNode(*node.b, values);
        case '*': return lhs * evalNode(*node.b, values);
        case '/': return lhs / evalNode(*node.b, values);
        case '^':
        {
            long n;
            if (integerExponent(*node.b, n))
                return powInt(lhs, n);
            return std::pow(lhs, evalNode(*node.b, values));
        }
        }
    }
    }
    return {};
}

double powIntReal(double base, long n)
{
    double r = 1.0;
    for (long k = 0; k < std::labs(n); ++k)
        r *= base;
    return n < 0 ? 1.0 / r : r;
}

// Real-line twin of evalNode: same AST, doubles instead of std::complex. Real ODE
// right-hand sides get evaluated thousands of times per frame by an Integrator, so
// this path skips the complex arithmetic entirely. Constants keep only their real
// part, so `i` reads as 0 here.
double evalRealNode(const Node& node, const std::vector<double>& values)
{
    switch (node.kind)
    {
    case Node::Kind::Constant: return node.value.real();
    case Node::Kind::Variable: return values.at(node.index);
    case Node::Kind::Unary:    return -evalRealNode(*node.a, values);
    case Node::Kind::Call:     return kFuncs[node.index].evalR(evalRealNode(*node.a, values));
    case Node::Kind::Binary:
    {
        double lhs = evalRealNode(*node.a, values);
        switch (node.op)
        {
        case '+': return lhs + evalRealNode(*node.b, values);
        case '-': return lhs - evalRealNode(*node.b, values);
        case '*': return lhs * evalRealNode(*node.b, values);
        case '/': return lhs / evalRealNode(*node.b, values);
        case '^':
        {
            long n;
            if (integerExponent(*node.b, n))
                return powIntReal(lhs, n);
            return std::pow(lhs, evalRealNode(*node.b, values));
        }
        }
    }
    }
    return 0.0;
}

bool usesVariableNode(const Node& node, int index)
{
    if (node.kind == Node::Kind::Variable)
        return node.index == index;
    if (node.a && usesVariableNode(*node.a, index))
        return true;
    return node.b && usesVariableNode(*node.b, index);
}

// ---- GLSL codegen -----------------------------------------------------------

std::string glslFloat(double v)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    std::string s = buf;
    if (s.find_first_of(".eE") == std::string::npos)
        s += ".0";
    return s;
}

std::string glslNode(const Node& node, const std::vector<std::string>& glslVars)
{
    switch (node.kind)
    {
    case Node::Kind::Constant:
        return "vec2(" + glslFloat(node.value.real()) + ", " + glslFloat(node.value.imag()) + ")";
    case Node::Kind::Variable:
        return glslVars.at(node.index);
    case Node::Kind::Unary:
        return "(-" + glslNode(*node.a, glslVars) + ")";
    case Node::Kind::Call:
    {
        std::string pattern = kFuncs[node.index].glslPattern;
        std::string arg = glslNode(*node.a, glslVars);
        std::string out;
        for (char c : pattern)
            if (c == '$')
                out += arg;
            else
                out += c;
        return out;
    }
    case Node::Kind::Binary:
    {
        std::string lhs = glslNode(*node.a, glslVars);
        switch (node.op)
        {
        case '+': return "(" + lhs + " + " + glslNode(*node.b, glslVars) + ")";
        case '-': return "(" + lhs + " - " + glslNode(*node.b, glslVars) + ")";
        case '*': return "cMul(" + lhs + ", " + glslNode(*node.b, glslVars) + ")";
        case '/': return "cDiv(" + lhs + ", " + glslNode(*node.b, glslVars) + ")";
        case '^':
        {
            long n;
            if (integerExponent(*node.b, n))
                return "cPowi(" + lhs + ", " + std::to_string(n) + ")";
            return "cPow(" + lhs + ", " + glslNode(*node.b, glslVars) + ")";
        }
        }
    }
    }
    return "vec2(0.0, 0.0)";
}

// ---- LaTeX printer ----------------------------------------------------------

// Binding strength of a printed node, for deciding where parentheses are needed:
// sums 1, products 2, unary minus 3, powers 4, atoms (numbers, variables, calls,
// fractions) 5. A negative constant prints with its sign, so it binds like a unary.
int texPrec(const Node& node)
{
    switch (node.kind)
    {
    case Node::Kind::Constant:
        return node.value.real() < 0.0 || node.value.imag() != 0.0 ? 3 : 5;
    case Node::Kind::Variable:
    case Node::Kind::Call:
        return 5;
    case Node::Kind::Unary:
        return 3;
    case Node::Kind::Binary:
        switch (node.op)
        {
        case '+': case '-': return 1;
        case '*':           return 2;
        case '/':           return 5; // \frac{}{} is self-delimiting
        case '^':           return 4;
        }
    }
    return 5;
}

// Plain parentheses unless the inside is taller than a line of text (a fraction):
// \left( \right) adds inner spacing that reads as "sin (x)".
std::string texParen(const std::string& s)
{
    if (s.find("\\frac") != std::string::npos)
        return "\\left(" + s + "\\right)";
    return "(" + s + ")";
}

std::string texReal(double v)
{
    if (std::abs(v - kPi) < 1e-12) return "\\pi";
    if (std::abs(v - 2.0 * kPi) < 1e-12) return "\\tau";
    if (std::abs(v - 2.71828182845904523536) < 1e-12) return "e";
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

// A sign is written {-}: a bare '-' at the start of a row or group is spaced as
// subtraction ("- sin(x)"), a braced one hugs its operand.
std::string texConstant(cd v)
{
    if (v.imag() == 0.0)
        return v.real() < 0.0 ? "{-}" + texReal(-v.real()) : texReal(v.real());
    if (v.real() == 0.0)
        return v.imag() == 1.0 ? "i" : texReal(v.imag()) + "i";
    return texParen(texReal(v.real()) + (v.imag() < 0.0 ? " - " : " + ") +
                    texReal(std::abs(v.imag())) + "i");
}

std::string texNode(const Node& node, const std::vector<std::string>& vars);

// `node` printed as an operand that must bind at least as tightly as `minPrec`.
std::string texOperand(const Node& node, const std::vector<std::string>& vars, int minPrec)
{
    std::string s = texNode(node, vars);
    return texPrec(node) < minPrec ? texParen(s) : s;
}

// Leftmost printed token is a digit: then a product needs an explicit \cdot, or
// 2 * 3 would read as 23.
bool texStartsWithNumber(const Node& node)
{
    switch (node.kind)
    {
    case Node::Kind::Constant:
        return node.value.imag() == 0.0 && node.value.real() >= 0.0 &&
               std::isdigit((unsigned char)texReal(node.value.real())[0]);
    case Node::Kind::Binary:
        return node.op != '/' && texPrec(*node.a) >= texPrec(node) &&
               texStartsWithNumber(*node.a);
    default:
        return false;
    }
}

std::string texNode(const Node& node, const std::vector<std::string>& vars)
{
    switch (node.kind)
    {
    case Node::Kind::Constant:
        return texConstant(node.value);
    case Node::Kind::Variable:
        return vars.at(node.index);
    case Node::Kind::Unary:
        return "{-}" + texOperand(*node.a, vars, 4);
    case Node::Kind::Call:
    {
        std::string arg = texNode(*node.a, vars);
        const std::string name = kFuncs[node.index].name;
        if (name == "exp")  return "e^{" + arg + "}";
        if (name == "sqrt") return "\\sqrt{" + arg + "}";
        if (name == "abs")  return "\\left|" + arg + "\\right|";
        if (name == "conj") return "\\overline{" + arg + "}";
        if (name == "log" || name == "ln") return "\\ln" + texParen(arg);
        if (name == "re")   return "\\mathrm{Re}" + texParen(arg);
        if (name == "im")   return "\\mathrm{Im}" + texParen(arg);
        return "\\" + name + texParen(arg); // sin cos tan sinh cosh tanh arg
    }
    case Node::Kind::Binary:
        switch (node.op)
        {
        case '+':
            // a + (-b) reads as a - b
            if (node.b->kind == Node::Kind::Unary)
                return texNode(*node.a, vars) + " - " + texOperand(*node.b->a, vars, 2);
            return texNode(*node.a, vars) + " + " + texOperand(*node.b, vars, 2);
        case '-':
            return texNode(*node.a, vars) + " - " + texOperand(*node.b, vars, 2);
        case '*':
        {
            std::string lhs = texOperand(*node.a, vars, 2);
            std::string rhs = texOperand(*node.b, vars, 4); // -x or -3 on the right: parens
            if (texPrec(*node.b) >= 4 && texStartsWithNumber(*node.b))
                return lhs + " \\cdot " + rhs;
            // 2x and -0.3x read naturally; after a symbol or \pi a thin space keeps
            // x\,y apart (and \pi\,x from becoming the unknown command \pix)
            const Node* num = node.a.get();
            if (num->kind == Node::Kind::Unary)
                num = num->a.get();
            if (num->kind == Node::Kind::Constant && texStartsWithNumber(*num))
                return lhs + rhs;
            if (rhs[0] == '(' || rhs.rfind("\\left(", 0) == 0)
                return lhs + rhs; // x(28 - z): the parenthesis already separates

            return lhs + "\\," + rhs;
        }
        case '/':
            return "\\frac{" + texNode(*node.a, vars) + "}{" + texNode(*node.b, vars) + "}";
        case '^':
        {
            // a call or fraction as the base still needs parentheses: sin(x)^2
            std::string base = texNode(*node.a, vars);
            if (texPrec(*node.a) < 5 || node.a->kind == Node::Kind::Call ||
                (node.a->kind == Node::Kind::Binary && node.a->op == '/'))
                base = texParen(base);
            return base + "^{" + texNode(*node.b, vars) + "}";
        }
        }
    }
    return "0";
}

} // namespace

// ---- Expression -------------------------------------------------------------

Expression::Expression() = default;
Expression::~Expression() = default;
Expression::Expression(Expression&&) noexcept = default;
Expression& Expression::operator=(Expression&&) noexcept = default;

Expression Expression::parse(const std::string& source,
                             const std::vector<std::string>& variables,
                             std::string& error)
{
    Expression expr;
    try
    {
        std::vector<Token> tokens = tokenize(source);
        Parser parser{tokens, variables};
        NodePtr root = parser.parseExpression();
        if (parser.peek().kind != Token::End)
            fail("unexpected trailing input", parser.peek().pos);
        expr.root = std::move(root);
        error.clear();
    }
    catch (const ParseError& e)
    {
        error = e.message;
        expr.root.reset();
    }
    return expr;
}

std::complex<double> Expression::eval(const std::vector<std::complex<double>>& values) const
{
    return root ? evalNode(*root, values) : cd{};
}

double Expression::evalReal(const std::vector<double>& values) const
{
    return root ? evalRealNode(*root, values) : 0.0;
}

bool Expression::usesVariable(int index) const
{
    return root && usesVariableNode(*root, index);
}

std::string Expression::glsl(const std::vector<std::string>& glslVars) const
{
    return root ? glslNode(*root, glslVars) : "vec2(0.0, 0.0)";
}

std::string Expression::latex(const std::vector<std::string>& texVars) const
{
    return root ? texNode(*root, texVars) : "?";
}

const char* Expression::glslComplexLib()
{
    // Canonical home of the vec2 complex helpers; keep in sync with the emit
    // patterns above. a = (ax, ay) represents ax + i*ay.
    return R"glsl(
vec2 cMul(vec2 a, vec2 b) {
    return vec2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

vec2 cDiv(vec2 a, vec2 b) {
    float denom = dot(b, b); // |b|^2
    return vec2(dot(a, b), a.y * b.x - a.x * b.y) / denom;
}

vec2 cConj(vec2 a) {
    return vec2(a.x, -a.y);
}

float cMod(vec2 a) {
    return length(a);
}

float cArg(vec2 a) {
    return atan(a.y, a.x);
}

vec2 cExp(vec2 a) {
    // e^(x+iy) = e^x * (cos y + i sin y)
    return exp(a.x) * vec2(cos(a.y), sin(a.y));
}

vec2 cLog(vec2 a) {
    // ln|a| + i*arg(a)
    return vec2(log(length(a)), atan(a.y, a.x));
}

vec2 cPow(vec2 a, vec2 b) {
    // a^b = e^(b * ln(a))
    return cExp(cMul(b, cLog(a)));
}

// Small integer powers by repeated multiplication: exact and finite at a = 0,
// unlike the exp/log route through cPow.
vec2 cPowi(vec2 a, int n) {
    vec2 r = vec2(1.0, 0.0);
    int m = n < 0 ? -n : n;
    for (int k = 0; k < m; ++k)
        r = cMul(r, a);
    return n < 0 ? cDiv(vec2(1.0, 0.0), r) : r;
}

vec2 cSqrt(vec2 a) {
    float r = length(a);
    return vec2(sqrt((r + a.x) / 2.0), sign(a.y) * sqrt((r - a.x) / 2.0));
}

vec2 cSin(vec2 a) {
    // sin(x+iy) = sin(x)cosh(y) + i cos(x)sinh(y)
    return vec2(sin(a.x) * cosh(a.y), cos(a.x) * sinh(a.y));
}

vec2 cCos(vec2 a) {
    // cos(x+iy) = cos(x)cosh(y) - i sin(x)sinh(y)
    return vec2(cos(a.x) * cosh(a.y), -sin(a.x) * sinh(a.y));
}

vec2 cTan(vec2 a) {
    return cDiv(cSin(a), cCos(a));
}

vec2 cSinh(vec2 a) {
    // sinh(x+iy) = sinh(x)cos(y) + i cosh(x)sin(y)
    return vec2(sinh(a.x) * cos(a.y), cosh(a.x) * sin(a.y));
}

vec2 cCosh(vec2 a) {
    // cosh(x+iy) = cosh(x)cos(y) + i sinh(x)sin(y)
    return vec2(cosh(a.x) * cos(a.y), sinh(a.x) * sin(a.y));
}

vec2 cTanh(vec2 a) {
    return cDiv(cSinh(a), cCosh(a));
}
)glsl";
}

} // namespace engine
