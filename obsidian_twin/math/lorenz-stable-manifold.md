---
type: math
status: wip
code:
  - sims/lorenz_manifold/LorenzManifoldSim.cpp
  - engine/src/InvariantManifold.cpp
---
# lorenz-stable-manifold

## The idea
The Lorenz system $\dot x = \sigma(y-x)$, $\dot y = \rho x - y - xz$, $\dot z = xy -
\beta z$ has an equilibrium at the origin whose Jacobian, at the classic parameters
$\sigma=10,\rho=28,\beta=8/3$, has one unstable eigenvalue and two stable ones. The
2-D **stable manifold** $W^s(0)$ — the "Lorenz manifold" — is the set of points that
flow *into* the origin under the forward dynamics. The **parametrization method**
(Cabré–Fontich–de la Llave) finds it directly as a power series map $K:\mathbb
R^2\to\mathbb R^3$ tangent to the stable eigenspace, without integrating any ODE.
[[surface-renderer-ray-marched]] renders level sets of an explicit $f$; this is the
opposite situation — the manifold has no closed-form $f$, so it's built as a
parametrized patch instead.

## The equations
Jacobian at the origin, eigenvalues, and stable eigenvectors:

$$A=\begin{pmatrix}-\sigma&\sigma&0\\\rho&-1&0\\0&0&-\beta\end{pmatrix},\qquad
\lambda_1=-\beta,\ \ \lambda_2=\frac{-(\sigma+1)-\sqrt{(\sigma+1)^2+4\sigma(\rho-1)}}{2}$$

$$e_1=(0,0,1),\qquad e_2=\left(\frac{\sigma}{\sigma+\lambda_2},\,1,\,0\right)$$

$K$ is sought as a Taylor series $K_c(\theta_1,\theta_2)=\sum_{m+n\le
\text{order}} p^c_{mn}\,\theta_1^m\theta_2^n$ for each channel $c\in\{x,y,z\}$,
solving the invariance equation $\dot K = f(K)$ order by order. Seeding
$p_{10}=e_1$, $p_{01}=e_2$, each higher order $d=m+n$ reduces to one 3×3 linear
solve per $(m,n)$:

$$(A-\mu I)\,p_{mn} = b_{mn},\qquad \mu = m\lambda_1+n\lambda_2$$

where $b_{mn}$ comes from the Cauchy product (2-D convolution) of already-known
lower-order coefficients against the system's nonlinear terms ($-xz$ feeds the $y$
row, $xy$ feeds the $z$ row) — this is the homological equation the method reduces
to. $A-\mu I$ is singular only if $\mu$ happens to equal an eigenvalue of $A$
(a resonance), which doesn't occur for these parameters.

Because $e_1\perp e_2$ but $|e_2|\ne 1$, equal steps in $(\theta_1,\theta_2)$ are not
equal length; isotropic coordinates $\theta_1=a,\ \theta_2=b/|e_2|$ make a unit
$(a,b)$-circle approximate a metric circle to $O(\text{radius}^2)$. Geodesic-distance
rings $C_i$ at $\rho_i = i\cdot d\rho$ are then just circles in $(a,b)$ pushed through
$K$.

## Where it's implemented
`sims/lorenz_manifold/LorenzManifoldSim.cpp`: `buildManifold` (eigendata + the
order-by-order coefficient recursion via `S` and `solve3x3`), `evalK` (Taylor
evaluation), `equalArcResample` (arc-length remeshing of a ring, step 5 of the
build). All in `double`; only the final GL vertex buffer is cast to `float`.

Generalized in [[InvariantManifold]] (any equilibrium of a quadratic field, stable or
unstable side, complex-conjugate pairs via variables $(z,\bar z)$); its test
reproduces this sim's coefficients after rescaling the eigenvectors. Porting the sim
onto the module is optional.

## Pitfalls
- $K$ is only accurate inside its Taylor series' radius of convergence; rings drawn
  past that radius visibly cross or shoot off. That's expected, not a bug — it's the
  signal to switch to flow-based growth (see below), not to raise `order` further.
- Equal-arc resampling (`equalArcResample`) uses Euclidean chord length in
  $\mathbb R^3$ as a stand-in for the induced-metric arc length — cheap and good
  enough near the origin, but a coarser approximation as $\rho_i$ grows.
- Sign/index conventions in the recursion are load-bearing (which channel feeds
  which row of $b_{mn}$); port changes to this code by re-deriving from the Lorenz
  vector field, not by pattern-matching neighboring lines.

## Future
Phase 2 (not yet implemented, scoped in the sim's original build prompt): grow the
mesh past the convergence disk by integrating the **reversed** field $\dot x=-f(x)$
(RK4) from each ring point, shooting for the next geodesic level via 2×2 Newton in a
transversal plane, with edge-based remeshing to keep spacing bounded. Keep behind a
flag so the Taylor-only Phase 1 path stays available as ground truth.
