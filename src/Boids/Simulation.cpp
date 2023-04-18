#include "Simulation.hpp"

Simulation::Simulation()
{
    // Initialize window
    int state = openGlInit();

    // Frame rate and misc
    srand(static_cast<unsigned int>(time(NULL)));
    _framerate = 60;
    _running = true;

    // Pointer to boids
    _boids = new Boid[BOIDS_COUNT];
    for (int i = 0; i < BOIDS_COUNT; i++) {
        int x = static_cast<int>(WALLOFFSET + rand() % static_cast<int>(SCR_WIDTH - WALLOFFSET * 2));
        int y = static_cast<int>(WALLOFFSET + rand() % static_cast<int>(SCR_HEIGHT - WALLOFFSET * 2));
        _boids[i] = Boid(glm::vec2{x, y}, static_cast<float>(SCR_WIDTH), 2);
    }

    _metadataSize = 4;
    _worldPosScaleAngleDegOffset = 6;
    _worldPosScaleAngleDegSize = BOIDS_COUNT * _worldPosScaleAngleDegOffset;
    _worldPosScaleAngleDegIdx1 = BOIDS_COUNT + _metadataSize;
    _worldPosScaleAngleDegIdx2 = BOIDS_COUNT + _metadataSize + _worldPosScaleAngleDegSize;
    _bufferSize = BOIDS_COUNT + _metadataSize + _worldPosScaleAngleDegSize * 2;
    _sharedBuffer = new float[_bufferSize];
    std::memset(_sharedBuffer, 0, _bufferSize * sizeof(float));
    _sharedBuffer[BOIDS_COUNT] = static_cast<float>(BOIDS_COUNT);
    _sharedBuffer[BOIDS_COUNT + 1] = static_cast<float>(SCR_WIDTH);
    _sharedBuffer[BOIDS_COUNT + 2] = static_cast<float>(SCR_HEIGHT);
    _bufferSelectorIdx = BOIDS_COUNT + 3;
    _sharedBuffer[_bufferSelectorIdx] = 1.0f;
    for (unsigned int j = 0, inc = BOIDS_COUNT + _metadataSize; j < 2; j++) {
        for (unsigned int i = 0; i < BOIDS_COUNT; i++, inc += _worldPosScaleAngleDegOffset) {
            _sharedBuffer[inc] = _boids[i].center.x;
            _sharedBuffer[inc + 1] = _boids[i].center.y;
            _sharedBuffer[inc + 2] = _boids[i].scale.x;
            _sharedBuffer[inc + 3] = _boids[i].scale.y;
            _sharedBuffer[inc + 4] = static_cast<float>(_boids[i].angleDeg);
            _sharedBuffer[inc + 5] = 0.0f; // hashKey
        }
    }
}

Simulation::~Simulation()
{
    delete[] _boids;
}

const char *Simulation::getFileContent(const std::string& path) const
{
    std::ifstream file(path);
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    char *cstr = new char[content.size() + 1];
    cstr = std::strcpy(cstr, content.c_str());

    return cstr;
}

void Simulation::compileShader(unsigned int *shaderId, std::string filename, unsigned int type)
{
    const char *shaderSource = getFileContent(std::string(SHADER_PATH) + filename);
    *shaderId = glCreateShader(type);
    glShaderSource(*shaderId, 1, &shaderSource, NULL);
    glCompileShader(*shaderId);

    int success;
    char infoLog[512];
    glGetShaderiv(*shaderId, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(*shaderId, 512, NULL, infoLog);
        std::string errorMsg = "ERROR::SHADER::COMPILATION_FAILED\n";
        errorMsg += infoLog;
        std::cout<<errorMsg<<std::endl;
    }

    delete[] shaderSource;
}

void Simulation::checkShaderProgramCompileError(unsigned int shaderProgramId)
{
    int  success;
    char infoLog[512];
    glGetProgramiv(shaderProgramId, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgramId, 512, NULL, infoLog);
        std::string errorMsg = "ERROR::SHADER::COMPILATION_FAILED\n";
        errorMsg += infoLog;
        std::cout<<errorMsg<<std::endl;
    }
}

int Simulation::openGlInit()
{
    // Start window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // glfw window creation
    // --------------------
    _window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (_window == NULL)
    {
        std::cout<< "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(_window);
    glfwSetFramebufferSizeCallback(_window, framebuffer_size_callback);

    // Compile shaders
    compileShader(&_vertexShader, "boid.vs", GL_VERTEX_SHADER);
    *GLint success;
    glGetShaderiv(_vertexShader, GL_COMPILE_STATUS, &success);

    if (success != GL_TRUE) {
        GLint log_size;
        glGetShaderiv(_vertexShader, GL_INFO_LOG_LENGTH, &log_size);
        char* shader_log = static_cast<char*>(malloc(log_size));
        glGetShaderInfoLog(_vertexShader, log_size, NULL, shader_log);
        std::cerr << "Fragment:" << shader_log << std::endl;
        return -1;
    }
    compileShader(&_fragmentShader, "boid.fs", GL_FRAGMENT_SHADER);
    _vertexFragProgram = glCreateProgram();
    glAttachShader(_vertexFragProgram, _vertexShader);
    glAttachShader(_vertexFragProgram, _fragmentShader);
    glLinkProgram(_vertexFragProgram);
    checkShaderProgramCompileError(_vertexFragProgram);
    glDeleteShader(_vertexShader);
    glDeleteShader(_fragmentShader);

    //compileShader(&_computeShader, "boid_flocking.comp", GL_COMPUTE_SHADER);
    //_computeProgramFlocking = glCreateProgram();
    //glAttachShader(_computeProgramFlocking, _computeShader);
    //glLinkProgram(_computeProgramFlocking);
    //checkShaderProgramCompileError(_computeProgramFlocking);
    //glDeleteShader(_computeShader);

    glGenBuffers(1, &_VBO);
    glGenBuffers(1, &_instanceVBO);
    glGenBuffers(1, &_SSBO);
    glGenVertexArrays(1, &_VAO);

    _projection = glm::mat4(1.0f);
    _projection = glm::ortho(0.0f, (float)SCR_WIDTH, (float)SCR_HEIGHT, 0.0f, -1.0f, 1.0f);
}


void Simulation::display()
{
    static int firstIt = 0;
    //glUseProgram(_computeProgramHashing);
    //    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _SSBO);
    //        glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * _bufferSize, &_sharedBuffer[0], GL_STREAM_DRAW);
    //        if (!firstIt) {
    //            delete[] _sharedBuffer;
    //            firstIt++;
    //        }
    //        glDispatchCompute(static_cast<GLuint>(glm::ceil(BUCKETS_COUNT / 32.0f)), 1, 1);
    //        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    //        _sharedBuffer = (float *)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
    //        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    //    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);
    //glUseProgram(0);

    glUseProgram(_computeProgramFlocking);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _SSBO);
            glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * _bufferSize, &_sharedBuffer[0], GL_STREAM_DRAW);
            glDispatchCompute(static_cast<GLuint>(glm::ceil(BOIDS_COUNT / 32.0f)), 1, 1);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            _sharedBuffer = (float *)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
            glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);
    glUseProgram(0);

    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(_vertexFragProgram);
        glUniformMatrix4fv(glGetUniformLocation(_vertexFragProgram, "projection"), 1, GL_FALSE, glm::value_ptr(_projection));
        Boid::prepareDrawingBuffers(_VAO, _VBO, _instanceVBO, _sharedBuffer[_bufferSelectorIdx] == 2.0f ? &_sharedBuffer[_worldPosScaleAngleDegIdx2] : &_sharedBuffer[_worldPosScaleAngleDegIdx1]);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 3, BOIDS_COUNT);
        Boid::clearDrawingBuffers(_VAO);
    glUseProgram(0);

}

void Simulation::run()
{
    double lastTime = clock();
    int nbFrames = 0;

    while (!glfwWindowShouldClose(_window))
    {
        double currentTime = clock();
        nbFrames++;
        if (currentTime - lastTime >= 1.0) {
            std::cout << nbFrames << " fps" << std::endl;
            nbFrames = 0;
            lastTime += 1.0;
        }
        display();
        // GL params
        _wireframe = false;
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_DEPTH);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(_window);
        glfwPollEvents();
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();

    glDeleteBuffers(1, &_VBO);
    glDeleteBuffers(1, &_instanceVBO);
    glDeleteBuffers(1, &_SSBO);

}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}


