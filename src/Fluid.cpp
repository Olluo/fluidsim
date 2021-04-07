#include "Fluid.h"

#include <ngl/MultiBufferVAO.h>
#include <ngl/NGLStream.h>
#include <ngl/Random.h>
#include <ngl/SimpleVAO.h>
#include <ngl/Util.h>
#include <ngl/VAOFactory.h>

Fluid::Fluid(uint32_t _w, uint32_t _h,
             uint32_t _numParticles,
             float diffusion, float viscosity, float dt) : m_width{_w},
                                                           m_height{_w},
                                                           m_numParticles{_numParticles},
                                                           m_size{N},
                                                           m_dt{dt},
                                                           m_diff{diffusion},
                                                           m_visc{viscosity},
                                                           m_s(N * N),
                                                           m_density(N * N),
                                                           m_Vx(N * N),
                                                           m_Vy(N * N),
                                                           m_Vx0(N * N),
                                                           m_Vy0(N * N)
{
    m_pos.resize(m_numParticles);
    m_dir.resize(m_numParticles);
    m_acceleration.resize(m_numParticles);
    m_maxspeed.resize(m_numParticles);
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
    diffuse(1, &m_Vx0, &m_Vx, m_visc, m_dt);
    diffuse(2, &m_Vy0, &m_Vy, m_visc, m_dt);

    project(&m_Vx0, &m_Vy0, &m_Vx, &m_Vy);

    advect(1, &m_Vx, &m_Vx0, &m_Vx0, &m_Vy0, m_dt);
    advect(2, &m_Vy, &m_Vy0, &m_Vx0, &m_Vy0, m_dt);

    project(&m_Vx, &m_Vy, &m_Vx0, &m_Vy0);

    diffuse(0, &m_s, &m_density, m_diff, m_dt);
    advect(0, &m_density, &m_s, &m_Vx, &m_Vy, m_dt);
}

void Fluid::addDensity(int _x, int _y, float _amount)
{
    m_density.at(IX(_x, _y)) += _amount;
}

void Fluid::addVelocity(int _x, int _y, float _amountX, float _amountY)
{
    int index = IX(_x, _y);

    m_Vx[index] += _amountX;
    m_Vy[index] += _amountY;
}

static void set_bnd(int _b, std::vector<float> *_x)
{
    for (int i = 1; i < N - 1; i++)
    {
        _x->at(IX(i, 0)) = _b == 2 ? -_x->at(IX(i, 1)) : _x->at(IX(i, 1));
        _x->at(IX(i, N - 1)) = _b == 2 ? -_x->at(IX(i, N - 2)) : _x->at(IX(i, N - 2));
    }
    for (int j = 1; j < N - 1; j++)
    {
        _x->at(IX(0, j)) = _b == 1 ? -_x->at(IX(1, j)) : _x->at(IX(1, j));
        _x->at(IX(N - 1, j)) = _b == 1 ? -_x->at(IX(N - 2, j)) : _x->at(IX(N - 2, j));
    }

    _x->at(IX(0, 0)) = 0.5f * (_x->at(IX(1, 0)) + _x->at(IX(0, 1)));
    _x->at(IX(0, N - 1)) = 0.5f * (_x->at(IX(1, N - 1)) + _x->at(IX(0, N - 2)));
    _x->at(IX(N - 1, 0)) = 0.5f * (_x->at(IX(N - 2, 0)) + _x->at(IX(N - 1, 1)));
    _x->at(IX(N - 1, N - 1)) = 0.5f * (_x->at(IX(N - 2, N - 1)) + _x->at(IX(N - 1, N - 2)));
}

static void lin_solve(int _b, std::vector<float> *_x, std::vector<float> *_x0, float _a, float _c)
{
    float cRecip = 1.0f / _c;
    for (int k = 0; k < iter; k++)
    {
        for (int j = 1; j < N - 1; j++)
        {
            for (int i = 1; i < N - 1; i++)
            {
                _x->at(IX(i, j)) =
                    (_x0->at(IX(i, j)) + _a * (_x->at(IX(i + 1, j)) + _x->at(IX(i - 1, j)) + _x->at(IX(i, j + 1)) + _x->at(IX(i, j - 1)))) * cRecip;
            }
        }

        set_bnd(_b, _x);
    }
}

static void diffuse(int _b, std::vector<float> *_x, std::vector<float> *_x0, float _diff, float _dt)
{
    float a = _dt * _diff * (N - 2) * (N - 2);
    lin_solve(_b, _x, _x0, a, 1 + 4 * a);
}

static void advect(int _b, std::vector<float> *_d, std::vector<float> *_d0, std::vector<float> *_velocX, std::vector<float> *_velocY, float _dt)
{
    float i0, i1, j0, j1;

    float dtx = _dt * (N - 2);
    float dty = _dt * (N - 2);

    float s0, s1, t0, t1;
    float tmp1, tmp2, x, y;

    float Nfloat = static_cast<float>(N);
    int i, j;
    float ifloat, jfloat;

    for (j = 1, jfloat = 1.0f; j < N - 1; j++, jfloat++)
    {
        for (i = 1, ifloat = 1.0f; i < N - 1; i++, ifloat++)
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

static void project(std::vector<float> *_velocX, std::vector<float> *_velocY, std::vector<float> *_p, std::vector<float> *_div)
{

    for (int j = 1; j < N - 1; j++)
    {
        for (int i = 1; i < N - 1; i++)
        {
            _div->at(IX(i, j)) = -0.5f * (_velocX->at(IX(i + 1, j)) - _velocX->at(IX(i - 1, j)) + _velocY->at(IX(i, j + 1)) - _velocY->at(IX(i, j - 1))) / N;
            _p->at(IX(i, j)) = 0;
        }
    }

    set_bnd(0, _div);
    set_bnd(0, _p);
    lin_solve(0, _p, _div, 1, 6);

    for (int j = 1; j < N - 1; j++)
    {
        for (int i = 1; i < N - 1; i++)
        {
            _velocX->at(IX(i, j)) -= 0.5f * (_p->at(IX(i + 1, j)) - _p->at(IX(i - 1, j))) * N;
            _velocY->at(IX(i, j)) -= 0.5f * (_p->at(IX(i, j + 1)) - _p->at(IX(i, j - 1))) * N;
        }
    }
    set_bnd(1, _velocX);
    set_bnd(2, _velocY);
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
        glDrawArrays(GL_POINTS, 0, m_numParticles);
        glBindVertexArray(0);
    }
}
void Fluid::update(float _dt)
{
    for (size_t i = 0; i < m_numParticles; ++i)
    {
        auto dir = m_dir[i] * ngl::Vec3(m_acceleration[i], 0.0f, m_acceleration[i]) * _dt;
        m_dir[i].clamp(m_maxspeed[i]);
        m_pos[i] += dir;
        if (m_maxspeed[i] <= 0.0f)
        {
            resetParticle(i);
        }

        float xsize = m_width / 2.0f;
        float zsize = m_height / 2.0f;
        // Now check against the bounds of the grid and reflect if needed this is quite brute force but works
        // left plane
        if (m_pos[i].m_x <= -xsize)
        {
            m_dir[i] = m_dir[i].reflect({1.0f, 0.0f, 0.0f});
            m_maxspeed[i] -= 0.1f;
        }
        // right plane
        else if (m_pos[i].m_x >= xsize)
        {
            m_dir[i] = m_dir[i].reflect({-1.0f, 0.0f, 0.0f});
            m_maxspeed[i] -= 0.1f;
        }
        // top plane
        if (m_pos[i].m_z >= zsize)
        {
            m_dir[i] = m_dir[i].reflect({0.0f, 0.0f, 1.0f});
            m_maxspeed[i] -= 0.1f;
        }
        // bottom plane
        else if (m_pos[i].m_z <= -zsize)
        {
            m_dir[i] = m_dir[i].reflect({0.0f, 0.0f, -1.0f});
            m_maxspeed[i] -= 0.1f;
        }
    }
}

void Fluid::resetParticle(size_t i)
{
    m_pos[i].m_x = ngl::Random::randomNumber(m_width / 2.0f);
    m_pos[i].m_z = ngl::Random::randomNumber(m_height / 2.0f);
    m_pos[i].m_y = 0.0f; // just in case!
    m_dir[i] = ngl::Random::getRandomVec3() * 2.0f;
    m_dir[i].m_y = 0.0f; // this needs to be done as reflect is 3d
    m_maxspeed[i] = ngl::Random::randomPositiveNumber(5) + 0.1f;
    m_acceleration[i] = ngl::Random::randomPositiveNumber(5) + 0.1f;
}

void Fluid::initGrid()
{
    for (size_t i = 0; i < m_numParticles; ++i)
    {
        resetParticle(i);
    }
}