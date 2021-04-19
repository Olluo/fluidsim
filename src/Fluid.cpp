#include "Fluid.h"

#include <ngl/MultiBufferVAO.h>
#include <ngl/NGLStream.h>
#include <ngl/Random.h>
#include <ngl/SimpleVAO.h>
#include <ngl/Util.h>
#include <ngl/VAOFactory.h>

Fluid::Fluid(size_t _size, float diffusion, float viscosity, float dt) : m_numParticles{_size * _size},
                                                                         m_size{_size},
                                                                         m_dt{dt},
                                                                         m_diff{diffusion},
                                                                         m_visc{viscosity},
                                                                         m_Vx(_size * _size),
                                                                         m_Vy(_size * _size),
                                                                         m_v(_size * _size),
                                                                         m_Vx0(_size * _size),
                                                                         m_Vy0(_size * _size),
                                                                         m_v0(_size * _size),
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

    // Going to use a non NGL buffer see if it is quicker

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
}

void Fluid::step()
{
    std::vector<float> Vx;
    std::vector<float> Vy;
    std::vector<float> Vx0;
    std::vector<float> Vy0;

    diffuse();

    splitter(m_v, &Vx, &Vy);
    splitter(m_v0, &Vx0, &Vy0);

    // diffuse(1, &Vx0, &Vx);
    // diffuse(2, &Vy0, &Vy);

    project(&Vx0, &Vy0, &Vx, &Vy);

    advect(1, &Vx, &Vx0, &Vx0, &Vy0);
    advect(2, &Vy, &Vy0, &Vx0, &Vy0);

    project(&Vx, &Vy, &Vx0, &Vy0);

    updateParticles();

    merger(&m_v, Vx, Vy);
    merger(&m_v0, Vx0, Vy0);
}

void Fluid::addVelocity(ngl::Vec2 _pos, ngl::Vec2 _v)
{
    _pos.m_x = std::clamp(_pos.m_x, 0.0f, static_cast<ngl::Real>(m_size - 1));
    _pos.m_y = std::clamp(_pos.m_y, 0.0f, static_cast<ngl::Real>(m_size - 1));
    size_t index = IX(static_cast<size_t>(_pos.m_x), static_cast<size_t>(_pos.m_y));

    m_Vx[index] += _v.m_x;
    m_Vy[index] += _v.m_y;
    m_v[index] += _v;
}

void Fluid::resetVelocities()
{
    std::fill(m_Vx.begin(), m_Vx.end(), 0.0f);
    std::fill(m_Vy.begin(), m_Vy.end(), 0.0f);
    std::fill(m_v.begin(), m_v.end(), ngl::Vec2{});
    std::fill(m_v0.begin(), m_v0.end(), ngl::Vec2{});
}

void Fluid::set_bnd(int _b, std::vector<float> *_x)
{
    for (int i = 1; i < m_size - 1; i++)
    {
        _x->at(IX(i, 0)) = _b == 2 ? -_x->at(IX(i, 1)) : _x->at(IX(i, 1));
        _x->at(IX(i, m_size - 1)) = _b == 2 ? -_x->at(IX(i, m_size - 2)) : _x->at(IX(i, m_size - 2));
    }
    for (int j = 1; j < m_size - 1; j++)
    {
        _x->at(IX(0, j)) = _b == 1 ? -_x->at(IX(1, j)) : _x->at(IX(1, j));
        _x->at(IX(m_size - 1, j)) = _b == 1 ? -_x->at(IX(m_size - 2, j)) : _x->at(IX(m_size - 2, j));
    }

    _x->at(IX(0, 0)) = 0.5f * (_x->at(IX(1, 0)) + _x->at(IX(0, 1)));
    _x->at(IX(0, m_size - 1)) = 0.5f * (_x->at(IX(1, m_size - 1)) + _x->at(IX(0, m_size - 2)));
    _x->at(IX(m_size - 1, 0)) = 0.5f * (_x->at(IX(m_size - 2, 0)) + _x->at(IX(m_size - 1, 1)));
    _x->at(IX(m_size - 1, m_size - 1)) = 0.5f * (_x->at(IX(m_size - 2, m_size - 1)) + _x->at(IX(m_size - 1, m_size - 2)));
}

void Fluid::set_bnd(std::vector<ngl::Vec2> *_v)
{
    // set top and bottom rows to reflect in y
    for (int i = 1; i < m_size - 1; i++)
    {
        _v->at(IX(i, 0)).m_y = -_v->at(IX(i, 1)).m_y;
        _v->at(IX(i, m_size - 1)).m_y = -_v->at(IX(i, m_size - 2)).m_y;
    }

    // set left and right columns to reflect in x
    for (int j = 1; j < m_size - 1; j++)
    {
        _v->at(IX(0, j)).m_x = -_v->at(IX(1, j)).m_x;
        _v->at(IX(m_size - 1, j)).m_x = -_v->at(IX(m_size - 2, j)).m_x;
    }

    // set corners to reflect inwards
    _v->at(IX(0, 0)) = 0.5f * (_v->at(IX(1, 0)) + _v->at(IX(0, 1)));
    _v->at(IX(0, m_size - 1)) = 0.5f * (_v->at(IX(1, m_size - 1)) + _v->at(IX(0, m_size - 2)));
    _v->at(IX(m_size - 1, 0)) = 0.5f * (_v->at(IX(m_size - 2, 0)) + _v->at(IX(m_size - 1, 1)));
    _v->at(IX(m_size - 1, m_size - 1)) = 0.5f * (_v->at(IX(m_size - 2, m_size - 1)) + _v->at(IX(m_size - 1, m_size - 2)));
}

void Fluid::lin_solve(int _b, std::vector<float> *_x, std::vector<float> *_x0, float _a, float _c)
{
    float cRecip = 1.0f / _c;
    for (int k = 0; k < iter; k++)
    {
        for (int j = 1; j < m_size - 1; j++)
        {
            for (int i = 1; i < m_size - 1; i++)
            {
                _x->at(IX(i, j)) =
                    (_x0->at(IX(i, j)) + _a * (_x->at(IX(i + 1, j)) + _x->at(IX(i - 1, j)) + _x->at(IX(i, j + 1)) + _x->at(IX(i, j - 1)))) * cRecip;
            }
        }

        set_bnd(_b, _x);
    }
}

void Fluid::lin_solve(std::vector<ngl::Vec2> *_v, std::vector<ngl::Vec2> *_v0, float _a, float _c)
{
    float cRecip = 1.0f / _c;
    for (int k = 0; k < iter; k++)
    {
        for (int j = 1; j < m_size - 1; j++)
        {
            for (int i = 1; i < m_size - 1; i++)
            {
                _v->at(IX(i, j)) =
                    (_v0->at(IX(i, j)) + _a * (_v->at(IX(i + 1, j)) + _v->at(IX(i - 1, j)) + _v->at(IX(i, j + 1)) + _v->at(IX(i, j - 1)))) * cRecip;
            }
        }

        set_bnd(_v);
    }
}

void Fluid::diffuse(int _b, std::vector<float> *_x, std::vector<float> *_x0)
{
    float a = m_dt * m_visc * (m_size - 2) * (m_size - 2);
    lin_solve(_b, _x, _x0, a, 1 + 4 * a);
}

void Fluid::diffuse()
{
    float a = m_dt * m_visc * (m_size - 2) * (m_size - 2);
    lin_solve(&m_v0, &m_v, a, 1 + 4 * a);
}

void Fluid::advect(int _b, std::vector<float> *_d, std::vector<float> *_d0, std::vector<float> *_velocX, std::vector<float> *_velocY)
{
    float i0, i1, j0, j1;

    float dtx = m_dt * (m_size - 2);
    float dty = m_dt * (m_size - 2);

    float s0, s1, t0, t1;
    float tmp1, tmp2, x, y;

    float Nfloat = static_cast<float>(m_size);
    int i, j;
    float ifloat, jfloat;

    for (j = 1, jfloat = 1.0f; j < m_size - 1; j++, jfloat++)
    {
        for (i = 1, ifloat = 1.0f; i < m_size - 1; i++, ifloat++)
        {
            tmp1 = dtx * _velocX->at(IX(i, j));
            tmp2 = dty * _velocY->at(IX(i, j));
            x = ifloat - tmp1;
            y = jfloat - tmp2;

            if (x < 0.5f)
                x = 0.5f;
            if (x > Nfloat + 0.5f)
                x = Nfloat + 0.5f;
            i0 = floorf(x);
            i1 = i0 + 1.0f;
            if (y < 0.5f)
                y = 0.5f;
            if (y > Nfloat + 0.5f)
                y = Nfloat + 0.5f;
            j0 = floorf(y);
            j1 = j0 + 1.0f;

            s1 = x - i0;
            s0 = 1.0f - s1;
            t1 = y - j0;
            t0 = 1.0f - t1;

            int i0i = static_cast<int>(i0);
            int i1i = static_cast<int>(i1);
            int j0i = static_cast<int>(j0);
            int j1i = static_cast<int>(j1);

            _d->at(IX(i, j)) =
                s0 * (t0 * _d0->at(IX(i0i, j0i)) + t1 * _d0->at(IX(i0i, j1i))) +
                s1 * (t0 * _d0->at(IX(i1i, j0i)) + t1 * _d0->at(IX(i1i, j1i)));
        }
    }

    set_bnd(_b, _d);
}

void Fluid::project(std::vector<float> *_velocX, std::vector<float> *_velocY, std::vector<float> *_p, std::vector<float> *_div)
{

    for (int j = 1; j < m_size - 1; j++)
    {
        for (int i = 1; i < m_size - 1; i++)
        {
            _div->at(IX(i, j)) = -0.5f * (_velocX->at(IX(i + 1, j)) - _velocX->at(IX(i - 1, j)) + _velocY->at(IX(i, j + 1)) - _velocY->at(IX(i, j - 1))) / m_size;
            _p->at(IX(i, j)) = 0;
        }
    }

    set_bnd(0, _div);
    set_bnd(0, _p);
    lin_solve(0, _p, _div, 1, 6);

    for (int j = 1; j < m_size - 1; j++)
    {
        for (int i = 1; i < m_size - 1; i++)
        {
            _velocX->at(IX(i, j)) -= 0.5f * (_p->at(IX(i + 1, j)) - _p->at(IX(i - 1, j))) * m_size;
            _velocY->at(IX(i, j)) -= 0.5f * (_p->at(IX(i, j + 1)) - _p->at(IX(i, j - 1))) * m_size;
        }
    }
    set_bnd(1, _velocX);
    set_bnd(2, _velocY);
}

void Fluid::updateParticles()
{

    for (int j = 0; j < m_size; j++)
    {
        for (int i = 0; i < m_size; i++)
        {
            auto pos = m_pos[IX(i, j)];
            int x0 = static_cast<int>(floor(static_cast<float>(pos.m_x)));
            int y0 = static_cast<int>(floor(static_cast<float>(pos.m_y)));

            int x1 = static_cast<int>(ceil(static_cast<float>(pos.m_x)));
            int y1 = static_cast<int>(ceil(static_cast<float>(pos.m_y)));

            // int x1 = std::clamp(x0 + 1, 0, static_cast<int>(m_size));
            // int y1 = std::clamp(y0 + 1, 0, static_cast<int>(m_size));

            // x0 = std::clamp(x0 - 1, 0, static_cast<int>(m_size));
            // y0 = std::clamp(y0 - 1, 0, static_cast<int>(m_size));

            auto xVel = 0.25f * (m_Vx[IX(x0, y0)] + m_Vx[IX(x1, y0)] + m_Vx[IX(x0, y1)] + m_Vx[IX(x1, y1)]);
            auto yVel = 0.25f * (m_Vy[IX(x0, y0)] + m_Vy[IX(x1, y0)] + m_Vy[IX(x0, y1)] + m_Vy[IX(x1, y1)]);
            auto vel = 0.25f * (m_v[IX(x0, y0)] + m_v[IX(x1, y0)] + m_v[IX(x0, y1)] + m_v[IX(x1, y1)]);

            ngl::Vec3 velocity = ngl::Vec3{vel.m_x, 0.0f, vel.m_y};

            m_pos[IX(i, j)] += velocity;

            if (m_pos[IX(i, j)].m_x <= 0.1f)
            {
                m_pos[IX(i, j)].m_x = static_cast<ngl::Real>(m_size - 1);
            }

            if (m_pos[IX(i, j)].m_x >= m_size - 0.1f)
            {
                m_pos[IX(i, j)].m_x = static_cast<ngl::Real>(0.0f + 1);
            }

            if (m_pos[IX(i, j)].m_z <= 0.1f)
            {
                m_pos[IX(i, j)].m_z = static_cast<ngl::Real>(m_size - 1);
            }

            if (m_pos[IX(i, j)].m_z >= m_size - 0.1f)
            {
                m_pos[IX(i, j)].m_z = static_cast<ngl::Real>(0.0f + 1);
            }

            if (velocity.lengthSquared() != 0.0f)
            {
                velocity.normalize();
            }

            m_dir[IX(i, j)] = velocity / 5.0f;
        }
    }
}

void Fluid::resetParticle(size_t i, size_t j)
{
    m_pos[IX(i, j)] = ngl::Vec3{static_cast<ngl::Real>(i), 0.0f, static_cast<ngl::Real>(j)};
}

void Fluid::initGrid()
{
    for (int j = 0; j < m_size; j++)
    {
        for (int i = 0; i < m_size; i++)
        {
            resetParticle(i, j);
        }
    }
}

void Fluid::toggleDrawMode(DrawMode _mode)
{
    m_drawMode = _mode;
}

void Fluid::draw() const
{
    if (m_drawMode == DrawMode::MULTIBUFFER)
    {
        m_vao->bind();
        // going to get a pointer to the data and update using memcpy
        auto ptr = m_vao->mapBuffer(0, GL_READ_WRITE);
        memcpy(ptr, &m_pos[0].m_x, m_pos.size() * sizeof(ngl::Vec3));
        m_vao->unmapBuffer();

        ptr = m_vao->mapBuffer(1, GL_READ_WRITE);
        memcpy(ptr, &m_dir[0].m_x, m_dir.size() * sizeof(ngl::Vec3));
        m_vao->unmapBuffer();

        // now unbind
        m_vao->draw();
        m_vao->unbind();
    }
    else
    {
        glBindVertexArray(m_svao);
        // bind the buffer to copy the data
        glBindBuffer(GL_ARRAY_BUFFER, m_vboID);
        // copy the pos data
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_numParticles * sizeof(ngl::Vec3), &m_pos[0].m_x);
        // concatinate the dir data
        glBufferSubData(GL_ARRAY_BUFFER, m_numParticles * sizeof(ngl::Vec3), m_numParticles * sizeof(ngl::Vec3), &m_dir[0].m_x);
        // draw
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_numParticles));
        glBindVertexArray(0);
    }
}
