#ifndef BOID_HPP_
#define BOID_HPP_

#include "glm/ext.hpp"

#define WALLOFFSET -1.0

#define BOIDS_COUNT 10

class Boid {
    public:
        Boid();
        Boid(const glm::vec2 position, const float screenWidth, const int scale = 20);
        ~Boid();

        // Getters
        const glm::vec2 &getWorldPosition() const;
        const glm::vec2 &getScale() const;
        double getAngleDeg() const;

        // GPU buffers
        static void prepareDrawingBuffers(unsigned int VAO, unsigned int VBO, unsigned int instanceVBO, float *worldPosScaleAngleDeg);
        static void clearDrawingBuffers(const unsigned int VAO);

        glm::vec2 center;
        glm::vec2 scale;
        double angleDeg;

    protected:
    private:
        static const unsigned int _vSize;
        static float _vertices[];
        static float _screenWidth;
        static int _nextID;

        static int _cellWidth;
        static int _gridWidth;
        int _diameter;
        int _radius;
        int _id;

        void setVerticeModel(float x, float y, unsigned int i);
};

#endif /* !BOID_HPP_ */