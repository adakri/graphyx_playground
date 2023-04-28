#ifndef Simulation_HPP_
#define Simulation_HPP_

#include "Boids.hpp"
#include "kernels.hpp"
#include "glad/glad.h"
#include "GLFW/glfw3.h"

#include <time.h>
#include <chrono>
#include <map>
#include <fstream>
#include <ctime>
#include <iostream>
#include <filesystem>

//debug
#define printarr(fmt, dat, len)	for (int i = 0; i < len; i++) printf(fmt, dat[i])


#define SHADER_PATH "shaders/"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void key_callback(GLFWwindow *window,int key,int scancode,int action,int mods);

// settings
const unsigned int SCR_WIDTH = 1600;
const unsigned int SCR_HEIGHT = 900;


class Simulation {
    public:
        Simulation();
        ~Simulation();

        void run();

    protected:
    private:
        void events();
        void display();
        void display_cuda();
        void update();
        void updateHashtable();

        GLFWwindow* _window;
        bool _running;
        double _lastTime;
        double _currentTime;

        Boid *_boids;
        float *_sharedBuffer;
        unsigned int _bufferSize;
        unsigned int _tableSize;
        unsigned int _worldPosScaleAngleDegSize;
        unsigned int _worldPosScaleAngleDegIdx1;
        unsigned int _worldPosScaleAngleDegIdx2;
        unsigned int _worldPosScaleAngleDegOffset;
        unsigned int _metadataSize;
        unsigned int _bufferSelectorIdx;

        // For OpenGL
        const char *getFileContent(const std::string &path, int& shaderSource_string_length) const;
        void compileShader(unsigned int *shaderId, std::string filename, unsigned int type);
        void checkShaderProgramCompileError(unsigned int shaderProgramId);
        int openGlInit();

        bool _wireframe;
        unsigned int _VAO, _VBO, _instanceVBO, _SSBO;
        unsigned int _vertexFragProgram;
        unsigned int _computeProgramFlocking, _computeProgramHashing;
        unsigned int _vertexShader;
        unsigned int _computeShader;
        unsigned int _fragmentShader;
        unsigned int _geometryShader;

        glm::mat4 _projection;
};

#endif /* !Simulation_HPP_ */