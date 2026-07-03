// CMakeProjectTest.cpp : Defines the entry point for the application.

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm.hpp>
#include <vec4.hpp>
#include <iostream>
#include <format>
#include <shader.h>
#include <camera.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <shapes.h>


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window, glm::vec4& color);
void CheckForErrors(unsigned int shader); 

Shader* shaderPtr=nullptr;

unsigned int SRC_WIDTH = 800;
unsigned int SRC_HEIGHT = 600;





int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(SRC_WIDTH, SRC_HEIGHT, "OpenGL Window", nullptr, nullptr);
    if (window == nullptr) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback); //window resize callback

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    glViewport(0,0,SRC_WIDTH,SRC_HEIGHT);


    Shader shader(SHADER_DIR"vertex.glsl", SHADER_DIR"complexFragment.glsl");
    shaderPtr = &shader;
    shader.use();

    int width, height;
    glfwGetFramebufferSize(window,&width,&height);
    shader.setVec2("resolution", float(width),float(height));

    Shader axisShader(SHADER_DIR"vertex.glsl", SHADER_DIR"fragment.glsl");
    

    int sector = 100;
    int stack = 100;
    float radius = 2.5f;
    
    int vertexCount;
    float* vertices = Shapes::sphereVertices(sector, stack, radius,vertexCount);
    int indicesCount;
    unsigned int* indices = Shapes::sphereIndices(sector,stack,indicesCount);
    

    //VAO 1
    unsigned int VBO1, VAO1, EBO1;
    glGenVertexArrays(1, &VAO1);
    glGenBuffers(1, &VBO1);
    glGenBuffers(1, &EBO1);

    glBindVertexArray(VAO1); //we bind the vao1
    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, vertexCount*sizeof(float), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO1);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesCount*sizeof(unsigned int), indices, GL_STATIC_DRAW);

    //vertex atrrib
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0); //enables location=0 for the vertex shader 


    //VAO 2 axis
    glLineWidth(4.0f);

    unsigned int VBO2, VAO2, EBO2;
    glGenVertexArrays(1, &VAO2);
    glGenBuffers(1, &VBO2);
    glGenBuffers(1, &EBO2);

    float* axisVertices = Shapes::axisVertices();
    unsigned int* axisIndices = Shapes::axisIndices();

    
    glBindVertexArray(VAO2); //we bind the vao2
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);    
    glBufferData(GL_ARRAY_BUFFER, 3*6*sizeof(float), axisVertices, GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO2);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(unsigned int), axisIndices, GL_STATIC_DRAW);
    
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    
    glEnableVertexAttribArray(0); //enables location=0 for the vertex shader 

    

    

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //render wire frame


    glm::vec4 color{ 0.f, 0.f, 0.f, 1.0f };


    Camera camera;
    camera.mode = Camera::CameraMode::Orbit;
    

    glm::mat4 model = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f));
    glm::mat4 view = glm::mat4(1.0f);
    model = glm::rotate(model, glm::radians(-90.f), glm::vec3(1.f,0.0f,.0f));
    glm::mat4 projection=glm::perspective(glm::radians(75.0f), (float)SRC_WIDTH / (float)SRC_HEIGHT, 0.1f, 100.0f);
    
   
    
    shader.use();
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);

    axisShader.use();
    axisShader.setMat4("projection", projection);
    axisShader.setMat4("view", view);
    
    

    



    while (!glfwWindowShouldClose(window)) {
        processInput(window, color);

        glClearColor(color.r, color.g, color.b, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        shader.use();

        
        camera.processInput(window);
        view = camera.getViewMatrix();

        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        shader.setVec2("mouse", float(mx) / SRC_WIDTH, 1.0f - float(my) / SRC_HEIGHT);

        shader.setMat4("view",view);
        
        
        
        
        //transformMat4 for Sphere
        glBindVertexArray(VAO1);

        projection=glm::perspective(glm::radians(55.f), (float)SRC_WIDTH / (float)SRC_HEIGHT, 0.1f, 100.0f);
        shader.setMat4("projection", projection);

        shader.setMat4("model", model);
        glDrawElements(GL_TRIANGLES,indicesCount, GL_UNSIGNED_INT, 0); //the amount of vertices drawn starts at cero finishes at n


       //shader for axis



        axisShader.use();
        axisShader.setMat4("view", view);
        axisShader.setMat4("projection", projection);
        glBindVertexArray(VAO2);
        axisShader.setMat4("model", glm::scale(model, glm::vec3(1.f)));
        glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT,0);//the amount of vertices drawn starts at cero finishes at n

        

       
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

//Readjust viewport size when user adjusts windowc
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
    //std::string s = std::format("Readjusted size to {}x{}",width, height);
    //std::cout << s << "\n";
    SRC_WIDTH = width;
    SRC_HEIGHT = height;

    shaderPtr->use();
    shaderPtr->setVec2("resolution", float(width),float(height));
    shaderPtr->setMat4("projection", glm::perspective(glm::radians(45.0f), (float)SRC_WIDTH / (float)SRC_HEIGHT, 0.1f, 100.0f));
}
void processInput(GLFWwindow* window, glm::vec4& color) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window,true);
    }

    if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
        color.r = 1.0f;
    }

}






