/**
 * @file FluidGrid.h
 * @brief This class uses the Fluid solver and the NGL particle system to display a fluid to a screen
 * 
 * @copyright Copyright (c) 2021
 */

#ifndef FLUID_GRID_H_
#define FLUID_GRID_H_

#include <vector>

#include <ngl/AbstractVAO.h>
#include <ngl/Vec2.h>
#include <ngl/Vec3.h>

#include "Fluid.h"

class FluidGrid
{
public:
    /**
     * @brief Construct a Fluid Grid
     * 
     * @param _viscosity The viscosity of the fluid
     * @param _dt The timestep of each iteration
     */
    FluidGrid(float _viscosity, float _dt);
    /**
     * @brief Step through one iteration of the solver
     * 
     */
    void step();
    /**
     * @brief Add velocity to the grid
     * 
     * @param _pos The position to add velocity
     * @param _v The velocity vector
     */
    void addVelocity(ngl::Vec2 _pos, ngl::Vec2 _v);
    /**
     * @brief Reset the grid to default
     * 
     */
    void reset()
    {
        resetVelocities();
        initGrid();
    }
    /**
     * @brief Draw the grid
     * 
     */
    void draw() const;
    /**
     * @brief Get the Num Particles object
     * 
     * @return size_t 
     */
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