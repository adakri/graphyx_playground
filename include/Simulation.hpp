#ifndef Simulation_HPP_
#define Simulation_HPP_

#include "Boids.hpp"
#include "glm/glm.hpp"
#include "GLFW/glfw3.h"

#include <time.h>
#include <map>
#include <fstream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);

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
        void update();
        void updateHashtable();

        GLFWwindow* _window;
        int _framerate;
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
        const char *getFileContent(const std::string &path) const;
        void compileShader(unsigned int *shaderId, std::string filename, unsigned int type);
        void checkShaderProgramCompileError(unsigned int shaderProgramId);
        void openGlInit();
        void openGlDraw();

        bool _wireframe;
        unsigned int _VAO, _VBO, _instanceVBO, _SSBO;
        unsigned int _vertexFragProgram;
        unsigned int _computeProgramFlocking, _computeProgramHashing;
        unsigned int _vertexShader;
        unsigned int _computeShader;
        unsigned int _fragmentShader;
        glm::mat4 _projection;
};

#endif /* !Simulation_HPP_ */