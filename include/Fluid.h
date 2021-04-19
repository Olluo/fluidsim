#ifndef FLUID_H_
#define FLUID_H_
#include <cstdint>
#include <math.h>
#include <vector>

#include <ngl/AbstractVAO.h>
#include <ngl/Vec3.h>

const int iter = 4;

class Fluid
{
public:
    Fluid(size_t _size, float _diffusion, float _viscosity, float _dt);

    void step();
    void addDensity(int _x, int _y, float _amount);
    void addVelocity(int _x, int _y, float _amountX, float _amountY);
    void reset()
    {
        resetVelocities();
        initGrid();
    }
    void draw() const;
    enum class DrawMode
    {
        SINGLEBUFFER,
        MULTIBUFFER
    };
    void toggleDrawMode(DrawMode _mode);
    size_t getNumParticles() const { return m_numParticles; }

private:
    size_t m_size;
    float m_dt;
    float m_diff;
    float m_visc;

    std::vector<float> m_s;
    std::vector<float> m_density;

    std::vector<float> m_Vx;
    std::vector<float> m_Vy;

    std::vector<float> m_Vx0;
    std::vector<float> m_Vy0;

    size_t m_numParticles;
    std::unique_ptr<ngl::AbstractVAO> m_vao;
    GLuint m_svao;
    GLuint m_vboID;
    std::vector<ngl::Vec3> m_pos;
    std::vector<ngl::Vec3> m_dir;
    std::vector<float> m_acceleration;
    std::vector<float> m_maxspeed;
    DrawMode m_drawMode = DrawMode::SINGLEBUFFER;

    void initGrid();
    void resetVelocities();

    void resetParticle(size_t i, size_t j);
    void updateParticles();

    int IX(int _x, int _y)
    {
        return _x + _y * m_size;
    }

    void set_bnd(int _b, std::vector<float> *_x);

    void lin_solve(int b, std::vector<float> *_x, std::vector<float> *_x0, float _a, float _c);

    void diffuse(int _b, std::vector<float> *_x, std::vector<float> *_x0, float _diff, float _dt);

    void advect(int _b, std::vector<float> *_d, std::vector<float> *_d0, std::vector<float> *_velocX, std::vector<float> *_velocY, float _dt);

    void project(std::vector<float> *_velocX, std::vector<float> *_velocY, std::vector<float> *_p, std::vector<float> *_div);
};

#endif // !FLUID_H_