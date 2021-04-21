/**
 * @file FluidGrid.cpp
 * @brief This class uses the Fluid solver and the NGL particle system to display a fluid to a screen
 * 
 * @copyright Copyright (c) 2021
 */

#include "FluidGrid.h"

#include <ngl/MultiBufferVAO.h>
#include <ngl/NGLStream.h>
#include <ngl/Random.h>
#include <ngl/SimpleVAO.h>
#include <ngl/Util.h>
#include <ngl/VAOFactory.h>

FluidGrid::FluidGrid(float viscosity, float dt) : m_numParticles{c_size * c_size},
                                                  m_dt{dt},
                                                  m_visc{viscosity},
                                                  m_Vx(c_size * c_size),
                                                  m_Vy(c_size * c_size),
                                                  m_Vx0(c_size * c_size),
                                                  m_Vy0(c_size * c_size),
                                                  m_pos(m_numParticles),
                                                  m_dir(m_numParticles)
{
    initGrid();

    m_vao = ngl::VAOFactory::createVAO(ngl::multiBufferVAO, GL_POINTS);
    m_vao->bind();
    m_vao->setData(ngl::MultiBufferVAO::VertexData(m_pos.size() * sizeof(ngl::Vec3), m_pos[0].m_x));
    m_vao->setVertexAttributePointer(0, 3, GL_FLOAT, 0, 0);
    m_vao->setData(ngl::MultiBufferVAO::VertexData(m_dir.size() * sizeof(ngl::Vec3), m_dir[0].m_x));
    m_vao->setVertexAttributePointer(1, 3, GL_FLOAT, 0, 0);
    m_vao->setNumIndices(m_numParticles);
    m_vao->unbind();

    // Going to use a non NGL buffer as quicker
    glGenVertexArrays(1, &m_svao);
    glBindVertexArray(m_svao);
    glGenBuffers(1, &m_vboID);
    // now bind this to the VBO buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_vboID);
    // allocate the buffer data we need two lots of vec3 one for pos one for dir use dynamic as we update
    // per frame and it may be quicker
    glBufferData(GL_ARRAY_BUFFER, (m_numParticles * 2) * sizeof(ngl::Vec3), 0, GL_DYNAMIC_DRAW);
    // As we are using glBufferSubData later we can set these now and it will be the same.
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
    // The dir vec3 is going to be put at the end of the pos block
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<float *>(m_numParticles * sizeof(ngl::Vec3)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    resetVelocities();
}

void FluidGrid::step()
{
    diffuseX();
    diffuseY();

    projectForwards();

    advectX();
    advectY();

    projectBackwards();

    updateParticles();
}

void FluidGrid::addVelocity(ngl::Vec2 _pos, ngl::Vec2 _v)
{
    // clamp x and y so inside the grid
    size_t index = Fluid::IX(std::clamp(static_cast<size_t>(_pos.m_x), static_cast<size_t>(0), c_size - 1),
                             std::clamp(static_cast<size_t>(_pos.m_y), static_cast<size_t>(0), c_size - 1));

    m_Vx[index] += _v.m_x;
    m_Vy[index] += _v.m_y;
}

void FluidGrid::resetVelocities()
{
    std::fill(m_Vx.begin(), m_Vx.end(), 0.0f);
    std::fill(m_Vy.begin(), m_Vy.end(), 0.0f);

    // add a small initial velocity to show something on the grid
    addVelocity(ngl::Vec2{c_size / 2.0f, c_size / 2.0f}, ngl::Vec2{-.0001f, 0.0f});
}

void FluidGrid::updateParticles()
{

    for (int j = 0; j < c_size; j++)
    {
        for (int i = 0; i < c_size; i++)
        {
            auto pos = m_pos[Fluid::IX(i, j)];

            int x0 = static_cast<int>(floor(static_cast<float>(pos.m_x)));
            int y0 = static_cast<int>(floor(static_cast<float>(pos.m_z)));

            int x1 = static_cast<int>(ceil(static_cast<float>(pos.m_x)));
            int y1 = static_cast<int>(ceil(static_cast<float>(pos.m_z)));

            // Get average velocity of 4 adjacent points
            auto xVel = 0.25f * (m_Vx[Fluid::IX(x0, y0)] + m_Vx[Fluid::IX(x1, y0)] + m_Vx[Fluid::IX(x0, y1)] + m_Vx[Fluid::IX(x1, y1)]);
            auto yVel = 0.25f * (m_Vy[Fluid::IX(x0, y0)] + m_Vy[Fluid::IX(x1, y0)] + m_Vy[Fluid::IX(x0, y1)] + m_Vy[Fluid::IX(x1, y1)]);

            ngl::Vec3 velocity{xVel, 0.0f, yVel};

            pos += velocity * 0.2f;

            if (pos.m_x < 0.0f)
            {
                pos.m_x = static_cast<ngl::Real>(c_size - 1);
            }

            if (pos.m_x >= c_size - 1)
            {
                pos.m_x = static_cast<ngl::Real>(0.0f);
            }

            if (pos.m_z < 0.0f)
            {
                pos.m_z = static_cast<ngl::Real>(c_size - 1);
            }

            if (pos.m_z >= c_size - 1)
            {
                pos.m_z = static_cast<ngl::Real>(0.0f);
            }

            m_pos[Fluid::IX(i, j)] = pos;

            if (velocity.lengthSquared() != 0.0f)
            {
                velocity.normalize();
            }

            m_dir[Fluid::IX(i, j)] = velocity / 2.0f;
        }
    }
}

void FluidGrid::resetParticle(size_t i, size_t j)
{
    m_pos[Fluid::IX(i, j)] = ngl::Vec3{static_cast<ngl::Real>(i), 0.0f, static_cast<ngl::Real>(j)};
}

void FluidGrid::initGrid()
{
    for (int j = 0; j < c_size; j++)
    {
        for (int i = 0; i < c_size; i++)
        {
            resetParticle(i, j);
        }
    }
}

void FluidGrid::draw() const
{
    glBindVertexArray(m_svao);
    // bind the buffer to copy the data
    glBindBuffer(GL_ARRAY_BUFFER, m_vboID);
    // copy the pos data
    glBufferSubData(GL_ARRAY_BUFFER, 0, m_numParticles * sizeof(ngl::Vec3), &m_pos[0].m_x);
    // concatenate the dir data
    glBufferSubData(GL_ARRAY_BUFFER, m_numParticles * sizeof(ngl::Vec3), m_numParticles * sizeof(ngl::Vec3), &m_dir[0].m_x);
    // draw
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_numParticles));
    glBindVertexArray(0);
}

void FluidGrid::diffuseX()
{
    Fluid::diffuse(Fluid::Boundary::X, &m_Vx0, &m_Vx, m_visc, m_dt);
}

void FluidGrid::diffuseY()
{
    Fluid::diffuse(Fluid::Boundary::Y, &m_Vy0, &m_Vy, m_visc, m_dt);
}

void FluidGrid::projectForwards()
{
    Fluid::project(&m_Vx0, &m_Vy0, &m_Vx, &m_Vy);
}

void FluidGrid::advectX()
{
    Fluid::advect(Fluid::Boundary::X, &m_Vx, &m_Vx0, &m_Vx0, &m_Vy0, m_dt);
}

void FluidGrid::advectY()
{
    Fluid::advect(Fluid::Boundary::Y, &m_Vy, &m_Vy0, &m_Vx0, &m_Vy0, m_dt);
}

void FluidGrid::projectBackwards()
{
    Fluid::project(&m_Vx, &m_Vy, &m_Vx0, &m_Vy0);
}
