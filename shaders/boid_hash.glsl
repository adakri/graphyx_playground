#version 460 core

// Hashing grid
#define BUCKETS_COUNT 400

// 32 threads/lanes per bloc
layout (local_size_x = 32) in;

// Hash 
struct s_hash {
    float size;
    float startIdx;
};

// Boid as explaned in shared buffer
struct s_boid {
    float centerX;
    float centerY;
    float scaleX;
    float scaleY;
    float angleDeg;
    float hashKey;
};

// Shared buufer was binded to 0
// std430: https://stackoverflow.com/questions/73189196/diffrence-between-std140-and-std430-layout
layout(std430, binding = 0) buffer sharedBufferBoids {
    s_hash htable[BUCKETS_COUNT];
    float boidsCount;
    float windowWidth;
    float windowHeight;
    float bufferSelector;
    s_boid boids[];
};

void main()
{
    if (gl_GlobalInvocationID.x >= BUCKETS_COUNT) return;

    int oldBoidsBuffer = bufferSelector == 2.0f ? 0 : int(boidsCount);
    int newBoidsBuffer = bufferSelector == 2.0f ? int(boidsCount) : 0;

    for (int i = 0, sortIdx = int(htable[gl_GlobalInvocationID.x].startIdx); i < boidsCount; i++, oldBoidsBuffer++) {
        if (boids[oldBoidsBuffer].hashKey == gl_GlobalInvocationID.x) {
            boids[newBoidsBuffer + sortIdx] = boids[oldBoidsBuffer];
            sortIdx++;
        }
    }
}