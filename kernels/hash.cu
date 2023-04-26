// Hashing grid
#define BUCKETS_COUNT 400

struct s_hash {
    float size;
    float startIdx;
};

struct s_boid {
    float centerX;
    float centerY;
    float scaleX;
    float scaleY;
    float angleDeg;
    float hashKey;
};

template <typename T>
__global__
void hash(T* sharedBufferBoids)
{
    struct sharedBufferBoids {
        s_hash htable[BUCKETS_COUNT];
        float boidsCount;
        float windowWidth;
        float windowHeight;
        float bufferSelector;
        s_boid boids[];
    };
    // 1D blocks and thread organisation
    int blockidx = blockIdx.x;
    int threadidx = threadIdx.x;
    int element_id = (blockidx * blockDim.x) + threadidx;
    
    if (element_id >= BUCKETS_COUNT) return;

    int oldBoidsBuffer = bufferSelector == 2.0f ? 0 : int(boidsCount);
    int newBoidsBuffer = bufferSelector == 2.0f ? int(boidsCount) : 0;

    for (int i = 0, sortIdx = int(htable[element_id].startIdx); i < boidsCount; i++, oldBoidsBuffer++) {
        if (boids[oldBoidsBuffer].hashKey == element_id) {
            boids[newBoidsBuffer + sortIdx] = boids[oldBoidsBuffer];
            sortIdx++;
        }
    }
}