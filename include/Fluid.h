#ifndef FLUID_H_
#define FLUID_H_
#include <cstdint>
#include <math.h>
#include <vector>

#include <ngl/AbstractVAO.h>
#include <ngl/Vec2.h>
#include <ngl/Vec3.h>

const int c_iter = 4;
const size_t c_size = 100;

class Fluid
{
public:

    static void set_bnd(int _b, std::vector<float> *_x);

    static void lin_solve(int b, std::vector<float> *_x, std::vector<float> *_x0, float _a, float _c);

    static void diffuse(int _b, std::vector<float> *_x, std::vector<float> *_x0, float _diff, float _dt);

    static void advect(int _b, std::vector<float> *_d, std::vector<float> *_d0, std::vector<float> *_velocX, std::vector<float> *_velocY, float _dt);

    static void project(std::vector<float> *_velocX, std::vector<float> *_velocY, std::vector<float> *_p, std::vector<float> *_div);

    static size_t IX(size_t _x, size_t _y)
    {
        return _x + _y * c_size;
    }

private:
    Fluid() {}
};

#endif // !FLUID_H_