#include <cuda.h>
#include <math.h> 

// Hashing grid
#define BUCKETS_COUNT 400

#define GRID_WIDTH sqrt(BUCKETS_COUNT)

#define PI 3.1415926538

// Sim parameters
#define COHESION 40.0f
#define ALIGNMENT 25.0f
#define SEPARATION 3.0f
#define FOV 260
#define ROTATION_SPEED 4
#define SPEED 2
#define RAYCAST_RANGE 50

// TODO: Add template structs

// Shared Buffer container
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

struct Container {
    s_hash htable[BUCKETS_COUNT];
    float boidsCount;
    float windowWidth;
    float windowHeight;
    float bufferSelector;
    s_boid boids[];
};

//-------------------------------------------------------------------------------
// vec2 struct (not using thrust) 
struct vec2
{
    float x;
    float y;

    vec2() : x(0.0f), y(0.0f) { }
    vec2(float X, float Y) : x(X), y(Y){ }
    explicit vec2(float S) : x(S), y(S) { }
    vec2 operator + (const vec2 &rhs) const { return vec2(x + rhs.x, y + rhs.y); }
    vec2 operator * (const vec2 &rhs) const { return vec2(x * rhs.x, y * rhs.y); }
    vec2 operator - (const vec2 &rhs) const { return vec2(x - rhs.x, y - rhs.y); }
    vec2 operator * (const float s)  const  { return vec2(x * s, y * s); }
    vec2 operator / (const float s)  const  { return vec2(x / s, y / s); }

    vec2 &operator *= (const float s)   { *this = *this * s; return *this; }
    vec2 &operator += (const vec2 &rhs) { *this = *this + rhs; return *this; }
    vec2 &operator *= (const vec2 &rhs) { *this = *this * rhs; return *this; }
    vec2 &operator -= (const vec2 &rhs) { *this = *this - rhs; return *this; }

    float &operator [] (unsigned int i)             { return (&x)[i]; }
    const float &operator [] (unsigned int i) const { return (&x)[i]; }
};

//-----------------------------------------------------------
// Norm
float magnitudeVec2(const vec2 a, const vec2 b)
{
    return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

// Get vec2 boid direction
vec2 getBoidDirection(const float angleDeg, const float len) {
    float angleRad = angleDeg * PI / 180;
    return vec2(len * cos(angleRad), len * sin(angleRad));
}

// Boid intersection
bool intersects(vec2 A, vec2 B, vec2 C, vec2 D) {
    float det, gamma, lambda;
    det = (B.x - A.x) * (D.y - C.y) - (D.x - C.x) * (B.y - A.y);
    if (det == 0)
        return false;
    else {
        lambda = ((D.y - C.y) * (D.x - A.x) + (C.x - D.x) * (D.y - A.y)) / det;
        gamma = ((A.y - B.y) * (D.x - A.x) + (B.x - A.x) * (D.y - A.y)) / det;
        return (0 < lambda && lambda < 1) && (0 < gamma && gamma < 1);
    }
};

// Normalization op
vec2 normalizeVec2(const vec2 a, const vec2 b)
{
    float magnitude = magnitudeVec2(a, b);
    vec2 normalize = vec2(a.x - b.x, a.y - b.y);

    normalize.x /= magnitude;
    normalize.y /= magnitude;

    return normalize;
}

// :)
vec2 rotatePointAroundCenter(vec2 point, const vec2 center, const float angleDeg)
{
    float angleRad = angleDeg * PI / 180;

    float sinVal = sin(angleRad);
    float cosVal = cos(angleRad);

    point.x -= center.x;
    point.y -= center.y;

    float xnew = point.x * cosVal - point.y * sinVal;
    float ynew = point.x * sinVal + point.y * cosVal;

    point.x = xnew + center.x;
    point.y = ynew + center.y;

    return point;
}

// :)
int mappedRadToDeg(float rad)
{
    float deg = rad * 180.0 / PI;
    return int(deg + 360) % 360;
}

int mappedDegrees(int deg)
{
    return int(deg + 360) % 360;
}





// Protos
template <typename T> __global__ void hash(T* sharedBufferBoids_);