#ifndef NOISE_H
#define NOISE_H

#include <algorithm>
#include <random>

#include "vec.h"


template <int N>
class NoiseGenerator
{
public:
    virtual GLfloat Noise(const vec<N> &p) const = 0;
};


typedef uint64_t NoiseSeed;

template <int N>
class PerlinNoiseGenerator: public NoiseGenerator<N>
{
public:
    PerlinNoiseGenerator(const NoiseSeed seed);

    GLfloat Noise(const vec<N> &) const;
};

inline GLfloat PerlinFade(float t)
{
    return t * t * t * (t * (t * 6 - 15) + 10);
}
inline GLfloat Lerp(float t, float a0, float a1)
{
    return a0 + t * (a1 - a0);
}
inline GLfloat PerlinGradient3D(NoiseSeed _hash, const vec3 &dir)
{
    static vec3 grad3d[] = {{1.0f, 1.0f, 0.0f},
                            {-1.0f, 1.0f, 0.0f},
                            {1.0f, -1.0f, 0.0f},
                            {-1.0f, -1.0f, 0.0f},
                            {1.0f, 0.0f, 1.0f},
                            {-1.0f, 0.0f, 1.0f},
                            {1.0f, 0.0f, -1.0f},
                            {-1.0f, 0.0f, -1.0f},
                            {0.0f, 1.0f, 1.0f},
                            {0.0f, -1.0f, 1.0f},
                            {0.0f, 1.0f, -1.0f},
                            {0.0f, -1.0f, -1.0f},
                            {1.0f, 1.0f, 0.0f},
                            {-1.0f, 1.0f, 0.0f},
                            {0.0f, -1.0f, 1.0f},
                            {0.0f, -1.0f, -1.0f}};

    return Dot(grad3d[_hash & 0x0f], dir);
}

template <>
class PerlinNoiseGenerator<3> : public NoiseGenerator<3>
{
private:
    NoiseSeed mPermutations[512];
private:
    void Reseed(const NoiseSeed seed)
    {
        for (size_t i = 0; i < 256; i++)
        {
            mPermutations[i] = i;
        }

        std::shuffle(std::begin(mPermutations),
                     std::begin(mPermutations) + 256,
                     std::default_random_engine(seed));

        for (size_t i = 0; i < 256; i++)
        {
            mPermutations[i + 256] = mPermutations[i];
        }
    }
public:
    PerlinNoiseGenerator(const NoiseSeed seed)
    {
        Reseed(seed);
    }

    /**
     * Returns between -1.0f and 1.0f, 0.0f for integers
     */
    GLfloat Noise(const vec3 &p) const
    {
        const NoiseSeed X = NoiseSeed(floor(p.x)) & 0xff;
        const NoiseSeed Y = NoiseSeed(floor(p.y)) & 0xff;
        const NoiseSeed Z = NoiseSeed(floor(p.z)) & 0xff;

        GLfloat dx = p.x - floor(p.x),
                dy = p.y - floor(p.y),
                dz = p.z - floor(p.z);

        const GLfloat fx = PerlinFade(dx);
        const GLfloat fy = PerlinFade(dy);
        const GLfloat fz = PerlinFade(dz);

        GLfloat grad000 = PerlinGradient3D(mPermutations[mPermutations[mPermutations[X] + Y] + Z], {dx, dy, dz}),
                grad100 = PerlinGradient3D(mPermutations[mPermutations[mPermutations[X + 1] + Y] + Z], {dx - 1.0f, dy, dz}),
                grad010 = PerlinGradient3D(mPermutations[mPermutations[mPermutations[X] + Y + 1] + Z], {dx, dy - 1.0f, dz}),
                grad110 = PerlinGradient3D(mPermutations[mPermutations[mPermutations[X + 1] + Y + 1] + Z], {dx - 1.0f, dy - 1.0f, dz}),
                grad001 = PerlinGradient3D(mPermutations[mPermutations[mPermutations[X] + Y] + Z + 1], {dx, dy, dz - 1.0f}),
                grad101 = PerlinGradient3D(mPermutations[mPermutations[mPermutations[X + 1] + Y] + Z + 1], {dx - 1.0f, dy, dz - 1.0f}),
                grad011 = PerlinGradient3D(mPermutations[mPermutations[mPermutations[X] + Y + 1] + Z + 1], {dx, dy - 1.0f, dz - 1.0f}),
                grad111 = PerlinGradient3D(mPermutations[mPermutations[mPermutations[X + 1] + Y + 1] + Z + 1], {dx - 1.0f, dy - 1.0f, dz - 1.0f});

        return Lerp(fz, Lerp(fy, Lerp(fx, grad000, grad100), Lerp(fx, grad010, grad110)),
                        Lerp(fy, Lerp(fx, grad001, grad101), Lerp(fx, grad011, grad111)));
    }
};

#endif  // NOISE_H
