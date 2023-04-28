#include "kernels.hpp"

template <typename T>
__global__
void hash(T* sharedBufferBoids_)
{
    Container  sharedBufferBoids = sharedBufferBoids_;
    // 1D blocks and thread organisation
    int blockidx = blockIdx.x;
    int threadidx = threadIdx.x;
    int element_id = (blockidx * blockDim.x) + threadidx;
    
    if (element_id >= BUCKETS_COUNT) return;

    int oldBoidsBuffer = sharedBufferBoids.bufferSelector == 2.0f ? 0 : int(sharedBufferBoids.boidsCount);
    int newBoidsBuffer = sharedBufferBoids.bufferSelector == 2.0f ? int(sharedBufferBoids.boidsCount) : 0;

    for (int i = 0, sortIdx = int(htable[element_id].startIdx); i < sharedBufferBoids.boidsCount; i++, oldBoidsBuffer++) {
        if (boids[oldBoidsBuffer].hashKey == element_id) {
            boids[newBoidsBuffer + sortIdx] = boids[oldBoidsBuffer];
            sortIdx++;
        }
    }
}