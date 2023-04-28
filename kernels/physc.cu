#include "kernels.hpp"


// Is boid touching the border, bring back to the other side
template <typename T> 
__device__
void checkBorder(const int thisBoid, T* boids, T* windowWidth, T* windowHeight)
{
    if (boids[thisBoid].centerX > windowWidth)
        boids[thisBoid].centerX = 0;
    if (boids[thisBoid].centerX < 0)
        boids[thisBoid].centerX = windowWidth;

    if (boids[thisBoid].centerY > windowHeight)
        boids[thisBoid].centerY = 0;
    if (boids[thisBoid].centerY < 0)
        boids[thisBoid].centerY = windowHeight;
}

// Add rotation to boid
template <typename T> 
__device__
void processRotation(int resultAngle, int thisBoidAngle, int thisBoid, T* boids)
{
    int degreesDif = resultAngle - thisBoidAngle;

    if (abs(degreesDif) <= ROTATION_SPEED) return;
    int tmpRotationSpeed = ROTATION_SPEED;
    if (resultAngle < thisBoidAngle && thisBoidAngle - resultAngle <= 180) tmpRotationSpeed *= -1;
    if (resultAngle > thisBoidAngle && resultAngle- thisBoidAngle > 180) tmpRotationSpeed *= -1;

    boids[thisBoid].angleDeg += tmpRotationSpeed;
}

template <typename T> 
__device__
bool raycast(vec2 directionVec, int thisBoid, T* boids,  T* windowWidth, T* windowHeight)
{
    if (boids[thisBoid].centerY > windowHeight || boids[thisBoid].centerY < 0) {
        vec2 mapCenter = vec2(windowWidth / 2, windowHeight / 2);
        int resultInDegrees = mappedDegrees(int(atan2(mapCenter.y - boids[thisBoid].centerY, mapCenter.x - boids[thisBoid].centerX) * 180 / PI));
        boids[thisBoid].angleDeg = resultInDegrees + 90;
        return true;
    }

    const int size = 4;
    const T offset = 500;
    T* walls[size];
    walls[0] = vec4(0 - offset, 0, windowWidth + offset, 0);
    walls[2] = vec4(windowWidth + offset, windowHeight, 0 - offset, windowHeight);

    T raycastAngle = 2.0f;
    vec2 center = vec2(boids[thisBoid].centerX, boids[thisBoid].centerY);
    vec2 frontRay = center + directionVec * RAYCAST_RANGE;
    vec2 leftRay = frontRay;
    vec2 rightRay = frontRay;
    rightRay = rotatePointAroundCenter(rightRay, center, raycastAngle);
    leftRay = rotatePointAroundCenter(leftRay, center, -raycastAngle);
    bool isColliding = false;
    for (int i = 0; i < size; i++) {
        if (intersects(center, frontRay, walls[i].xy, walls[i].zw)
        || intersects(center, leftRay, walls[i].xy, walls[i].zw)
        || intersects(center, rightRay, walls[i].xy, walls[i].zw)) {
            isColliding = true;
            break;
        }
    }
    if (!isColliding) return false;

    int collisionL = 0;
    int collisionR = 0;
    for (T it = raycastAngle; it <= 180.0f; it += raycastAngle) {
        collisionL = 0;
        collisionR = 0;
        rightRay = rotatePointAroundCenter(rightRay, center, raycastAngle);
        leftRay = rotatePointAroundCenter(leftRay, center, -raycastAngle);
        for (int i = 0; i < size; i++) {
            if (intersects(center, rightRay, walls[i].xy, walls[i].zw))
                collisionR++;
            if (intersects(center, leftRay, walls[i].xy, walls[i].zw))
                collisionL++;
        }
        if (collisionR == 0) {
            boids[thisBoid].angleDeg += ROTATION_SPEED * 2;
            return true;
        }
        if (collisionL == 0) {
            boids[thisBoid].angleDeg -= ROTATION_SPEED * 2;
            return true;
        }
    }
    return true;
}

template <typename T> 
__device__
int findClosestInRangeAngle(T range, int sepCount, int cohCount, int aliCount, vec2 separationVec, vec2 cohesionVec, vec2 alignementVec)
{
    if (sepCount != 0 && SEPARATION == range) {
        return mappedDegrees(int(atan2(separationVec.y, separationVec.x) * 180 / PI));
    } else if (cohCount != 0 && COHESION == range) {
        return mappedDegrees(int(atan2(cohesionVec.y, cohesionVec.x) * 180 / PI));
    } else if (aliCount != 0 && ALIGNMENT == range) {
        return mappedDegrees(int(atan2(alignementVec.y, alignementVec.x) * 180 / PI));
    }
    return 0;
}

template <typename T>
__global__
void run(T* sharedBufferBoids_)
{
    Container  sharedBufferBoids = sharedBufferBoids_;
    // Map T array
    s_hash htable[BUCKETS_COUNT] = sharedBufferBoids.htable;
    T boidsCount = sharedBufferBoids.boidsCount;
    T windowWidth = sharedBufferBoids.windowWidth;
    T windowHeight = sharedBufferBoids.windowHeight;
    T bufferSelector = sharedBufferBoids.bufferSelector;
    s_boid boids[] = sharedBufferBoids.boids;
    // 1D blocks and thread organisation
    int blockidx = blockIdx.x;
    int threadidx = threadIdx.x;
    int element_id = (blockidx * blockDim.x) + threadidx;

    if (element_id >= boidsCount) return;

    int thisBoid = int(gl_GlobalInvocationID.x);
    if (bufferSelector == 2.0f) thisBoid += int(boidsCount);

    int startIdx = int(htable[int(boids[gl_GlobalInvocationID.x].hashKey)].startIdx);
    if (bufferSelector == 2.0f) startIdx += int(boidsCount);
    int maxIdx = startIdx + int(htable[int(boids[gl_GlobalInvocationID.x].hashKey)].size);

    T maxRange = COHESION > ALIGNMENT ? COHESION > SEPARATION ? COHESION : SEPARATION : ALIGNMENT > SEPARATION ? ALIGNMENT : SEPARATION;
    T minRange = COHESION < ALIGNMENT ? COHESION < SEPARATION ? COHESION : SEPARATION : ALIGNMENT < SEPARATION ? ALIGNMENT : SEPARATION;
    T midRange = 0.0f;
    if (COHESION != maxRange && COHESION != minRange) midRange = COHESION;
    else if (ALIGNMENT != maxRange && ALIGNMENT != minRange) midRange = ALIGNMENT;
    else midRange = SEPARATION;

    vec2 separationVec = vec2(0, 0);
    int sepCount = 0;
    vec2 cohesionVec = vec2(0, 0);
    int cohCount = 0;
    vec2 alignementVec = vec2(0, 0);
    int aliCount = 0;

    T* boids = boids;

    T magnitude = 0.0f;
    for (int i = startIdx; i < maxIdx; i++) {
        if (i == thisBoid) continue;
        magnitude = magnitudeVec2(vec2(boids[i].centerX, boids[i].centerY), vec2(boids[thisBoid].centerX, boids[thisBoid].centerY));
        if (mappedDegrees(int(boids[i].angleDeg - 90)) - mappedDegrees(int(boids[thisBoid].angleDeg - 90)) > FOV / 2 && mappedDegrees(int(boids[i].angleDeg - 90)) - mappedDegrees(int(boids[thisBoid].angleDeg - 90)) < 360 - FOV / 2)
            continue;
        if (magnitude < SEPARATION) {
            separationVec.x += boids[i].centerX;
            separationVec.y += boids[i].centerY;
            sepCount++;
        }
        if (magnitude < COHESION) {
            cohesionVec.x += boids[i].centerX;
            cohesionVec.y += boids[i].centerY;
            cohCount++;
        }
        if (magnitude < ALIGNMENT) {
            alignementVec += getBoidDirection(boids[i].angleDeg - 90, 1.0f);
            aliCount++;
        }
    }
    if (sepCount != 0) {
        separationVec.x = separationVec.x / sepCount - boids[thisBoid].centerX;
        separationVec.y = separationVec.y / sepCount - boids[thisBoid].centerY;
        separationVec.x *= -1;
        separationVec.y *= -1;
    }
    if (cohCount != 0) {
        cohesionVec.x = cohesionVec.x / cohCount - boids[thisBoid].centerX;
        cohesionVec.y = cohesionVec.y / cohCount - boids[thisBoid].centerY;
    }
    if (aliCount != 0) {
        alignementVec.x /= aliCount;
        alignementVec.y /= aliCount;
    }

    vec2 directionVec = getBoidDirection(boids[thisBoid].angleDeg - 90, 1.0f);
    raycast<T>(directionVec, thisBoid, boids, windowWidth, windowHeight);
    
    int resultInDegrees = findClosestInRangeAngle(minRange, sepCount, cohCount, aliCount, separationVec, cohesionVec, alignementVec);
    if (resultInDegrees == 0)
        resultInDegrees = findClosestInRangeAngle(midRange, sepCount, cohCount, aliCount, separationVec, cohesionVec, alignementVec);
    if (resultInDegrees == 0)
       resultInDegrees = findClosestInRangeAngle(maxRange, sepCount, cohCount, aliCount, separationVec, cohesionVec, alignementVec);
    if (resultInDegrees != 0)
        processRotation<T>(resultInDegrees, mappedDegrees(int(boids[thisBoid].angleDeg - 90)), thisBoid, boids);
    
    directionVec = getBoidDirection(boids[thisBoid].angleDeg - 90, 1.0f);
    boids[thisBoid].centerX += directionVec.x * SPEED;
    boids[thisBoid].centerY += directionVec.y * SPEED;
    checkBorder<T>(thisBoid, boids, windowWidth, windowHeight);
}