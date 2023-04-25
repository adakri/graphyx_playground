#include "Simulation.hpp"

// Global state pause 
bool paused = false; 

Simulation::Simulation()
{
    std::cout<<"Initializing simulation..."<<std::endl;
    // Initialize window
    int state = openGlInit();


    // Frame rate and misc
    srand(static_cast<unsigned int>(time(NULL)));
    // Enforce framerate limit
    _framerate = 60;
    _running = true;

    _lastTime = 0.;

    // Pointer to boids
    _boids = new Boid[BOIDS_COUNT];
    for (int i = 0; i < BOIDS_COUNT; i++) {
        int x = static_cast<int>(WALLOFFSET + rand() % static_cast<int>(SCR_WIDTH - WALLOFFSET * 2));
        int y = static_cast<int>(WALLOFFSET + rand() % static_cast<int>(SCR_HEIGHT - WALLOFFSET * 2));
        _boids[i] = Boid(glm::vec2{x, y}, static_cast<float>(SCR_WIDTH), 3);
    }

    _tableSize = BUCKETS_COUNT * 2;
    _metadataSize = 4;
    _worldPosScaleAngleDegOffset = 6;
    _worldPosScaleAngleDegSize = BOIDS_COUNT * _worldPosScaleAngleDegOffset;
    _worldPosScaleAngleDegIdx1 = _tableSize + _metadataSize;
    _worldPosScaleAngleDegIdx2 = _tableSize + _metadataSize + _worldPosScaleAngleDegSize;
    _bufferSize = _tableSize + _metadataSize + _worldPosScaleAngleDegSize * 2;
    _sharedBuffer = new float[_bufferSize];
    std::memset(_sharedBuffer, 0, _bufferSize * sizeof(float));
    _sharedBuffer[_tableSize] = static_cast<float>(BOIDS_COUNT);
    _sharedBuffer[_tableSize + 1] = static_cast<float>(SCR_WIDTH);
    _sharedBuffer[_tableSize + 2] = static_cast<float>(SCR_HEIGHT);
    _bufferSelectorIdx = _tableSize + 3;
    _sharedBuffer[_bufferSelectorIdx] = 1.0f;
    for (unsigned int j = 0, inc = _tableSize + _metadataSize; j < 2; j++) {
        for (unsigned int i = 0; i < BOIDS_COUNT; i++, inc += _worldPosScaleAngleDegOffset) {
            _sharedBuffer[inc] = _boids[i].center.x;
            _sharedBuffer[inc + 1] = _boids[i].center.y;
            _sharedBuffer[inc + 2] = _boids[i].scale.x;
            _sharedBuffer[inc + 3] = _boids[i].scale.y;
            _sharedBuffer[inc + 4] = static_cast<float>(_boids[i].angleDeg);
            _sharedBuffer[inc + 5] = 0.0f; // hashKey
        }
    }
    //printarr("%.2f \n", _sharedBuffer, 30);
}

Simulation::~Simulation()
{
    delete[] _boids;
}

const char *Simulation::getFileContent(const std::string& path, int& shaderSource_string_length) const
{
    // Check if the file exists
    if( !std::filesystem::exists(path))
    {
        std::cout<<"File does not exist!"<<std::endl;
    }
    std::ifstream file(path);
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    char *cstr = new char[content.size() + 1];
    cstr = std::strcpy(cstr, content.c_str());
    // For debug purposes, string length
    shaderSource_string_length = content.length();

    return cstr;
}

void Simulation::compileShader(unsigned int *shaderId, std::string filename, unsigned int type)
{
    int shaderSource_string_length;
    const char *shaderSource = getFileContent(std::string(SHADER_PATH) + filename, shaderSource_string_length);
    unsigned int v = glCreateShader(type);
    *shaderId = glCreateShader(type);
    glShaderSource(*shaderId, 1, &shaderSource, &shaderSource_string_length);
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
    std::cout<<"Starting the initialization..."<<std::endl;
    int check = glfwInit();
    if( !check ) {
        std::cout<<"Failed to initialize GLFW"<<std::endl;
        return -1;
    }
    
    // Enabaling computer shaders
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    // glfw window creation
    // --------------------
    _window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Boids", NULL, NULL);
    if (_window == NULL)
    {
        std::cout<< "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(_window);
    glfwSetFramebufferSizeCallback(_window, framebuffer_size_callback);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // Compile shaders
    std::cout<<"Compiling shaders..."<<std::endl;
    std::cout<<"Vertex shader..."<<std::endl;
    compileShader(&_vertexShader, "boid.vs", GL_VERTEX_SHADER);
    std::cout<<"Frag shader..."<<std::endl;
    compileShader(&_fragmentShader, "boid.fs", GL_FRAGMENT_SHADER);
    _vertexFragProgram = glCreateProgram();
    glAttachShader(_vertexFragProgram, _vertexShader);
    glAttachShader(_vertexFragProgram, _fragmentShader);
    glLinkProgram(_vertexFragProgram);
    checkShaderProgramCompileError(_vertexFragProgram);
    glDeleteShader(_vertexShader);
    glDeleteShader(_fragmentShader);

    std::cout<<"Comp shader..."<<std::endl;
    compileShader(&_computeShader, "boid.cs", GL_COMPUTE_SHADER);
    _computeProgramFlocking = glCreateProgram();
    glAttachShader(_computeProgramFlocking, _computeShader);
    glLinkProgram(_computeProgramFlocking);
    checkShaderProgramCompileError(_computeProgramFlocking);
    glDeleteShader(_computeShader);

    compileShader(&_computeShader, "boid_hash.glsl", GL_COMPUTE_SHADER);
    _computeProgramHashing = glCreateProgram();
    glAttachShader(_computeProgramHashing, _computeShader);
    glLinkProgram(_computeProgramHashing);
    checkShaderProgramCompileError(_computeProgramHashing);
    glDeleteShader(_computeShader);

    std::cout<<"Generating buffers..."<<std::endl;
    glGenBuffers(1, &_VBO);
    glGenBuffers(1, &_instanceVBO);
    glGenBuffers(1, &_SSBO);
    glGenVertexArrays(1, &_VAO);

    _projection = glm::mat4(1.0f);
    _projection = glm::ortho(0.0f, (float)SCR_WIDTH, (float)SCR_HEIGHT, 0.0f, -1.0f, 1.0f);
}


void Simulation::display()
{
    Boid::updateHashtable(_sharedBuffer, _tableSize, _sharedBuffer[_bufferSelectorIdx] == 1.0f ? &_sharedBuffer[_worldPosScaleAngleDegIdx1] : &_sharedBuffer[_worldPosScaleAngleDegIdx2], _worldPosScaleAngleDegOffset, &_sharedBuffer[_bufferSelectorIdx]);
    static int firstIt = 0;
    // Compute shader code
    glUseProgram(_computeProgramHashing);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _SSBO);
            glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * _bufferSize, &_sharedBuffer[0], GL_STREAM_DRAW);
            if (!firstIt) {
                delete[] _sharedBuffer;
                firstIt++;
            }
            glDispatchCompute(static_cast<GLuint>(glm::ceil(BUCKETS_COUNT / 32.0f)), 1, 1);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            _sharedBuffer = (float *)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
            glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);
    glUseProgram(0);
    glUseProgram(_computeProgramFlocking);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _SSBO);
            glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * _bufferSize, &_sharedBuffer[0], GL_STREAM_DRAW);
            glDispatchCompute(static_cast<GLuint>(glm::ceil(BOIDS_COUNT / 32.0f)), 1, 1);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            _sharedBuffer = (float *)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
            glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);
    glUseProgram(0);

    //printarr("%.2f -", _sharedBuffer, 30);

    // Generate color and draw framebuffer
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
    std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
    int nbFrames = 0;

    while (!glfwWindowShouldClose(_window))
    {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        nbFrames++;
        if ((double)std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastTime).count() >= 1.0) 
        {
            std::cout << nbFrames << " fps" << std::endl;
            nbFrames = 0;
            lastTime = std::chrono::steady_clock::now();
        }
        if(!paused)
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
        glfwSetKeyCallback(_window, key_callback);
        glfwSwapBuffers(_window);
        glfwPollEvents();
    }

    glDeleteBuffers(1, &_VBO);
    glDeleteBuffers(1, &_instanceVBO);
    glDeleteBuffers(1, &_SSBO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();

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

// Pause resume callback
void key_callback(GLFWwindow *window,int key,int scancode,int action,int mods)
{
    if (key==GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(window,GL_TRUE);
    }
    if(key==GLFW_KEY_P && action == GLFW_PRESS){
        if(!paused){
            std::cout<<"Pausing"<<std::endl;
            paused = true;
        }else{
            std::cout<<"Resuming"<<std::endl;
            paused = false;
        }
    }
}



