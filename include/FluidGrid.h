#ifndef FLUID_GRID_H_
#define FLUID_GRID_H_
#include <cstdint>
#include <math.h>
#include <vector>

#include <ngl/AbstractVAO.h>
#include <ngl/Vec2.h>
#include <ngl/Vec3.h>

#include "Fluid.h"

class FluidGrid
{
public:
    FluidGrid(float _viscosity, float _dt);

    void step();
    void addVelocity(ngl::Vec2 _pos, ngl::Vec2 _v);
    void reset()
    {
        resetVelocities();
        initGrid();
    }
    void draw() const;
    size_t getNumParticles() const { return m_numParticles; }

private:
    float m_dt;
    float m_diff;
    float m_visc;

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

    void initGrid();
    void resetVelocities();

    void resetParticle(size_t i, size_t j);
    void updateParticles();

    void diffuseX();
    void diffuseY();
    void projectForwards();
    void advectX();
    void advectY();
    void projectBackwards();
};

#endif // !FLUID_GRID_H_