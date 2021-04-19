#ifndef FLUID_H_
#define FLUID_H_
#include <cstdint>
#include <math.h>
#include <vector>

#include <ngl/AbstractVAO.h>
#include <ngl/Vec2.h>
#include <ngl/Vec3.h>

const int iter = 4;

class Fluid
{
public:
    Fluid(size_t _size, float _diffusion, float _viscosity, float _dt);

    void step();
    void addVelocity(ngl::Vec2 _pos, ngl::Vec2 _v);
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

    // std::vector<float> m_s;
    // std::vector<float> m_density;

    std::vector<float> m_Vx;
    std::vector<float> m_Vy;

    std::vector<ngl::Vec2> m_v;
    std::vector<ngl::Vec2> m_v0;

    std::vector<float> m_Vx0;
    std::vector<float> m_Vy0;

    size_t m_numParticles;
    std::unique_ptr<ngl::AbstractVAO> m_vao;
    GLuint m_svao;
    GLuint m_vboID;
    std::vector<ngl::Vec3> m_pos;
    std::vector<ngl::Vec3> m_dir;
    // std::vector<float> m_acceleration;
    // std::vector<float> m_maxspeed;
    DrawMode m_drawMode = DrawMode::SINGLEBUFFER;

    void initGrid();
    void resetVelocities();

    void resetParticle(size_t i, size_t j);
    void updateParticles();

    size_t IX(size_t _x, size_t _y)
    {
        return _x + _y * m_size;
    }

    void set_bnd(int _b, std::vector<float> *_x);

    void set_bnd(std::vector<ngl::Vec2> *_v);

    void lin_solve(int b, std::vector<float> *_x, std::vector<float> *_x0, float _a, float _c);

    void lin_solve(std::vector<ngl::Vec2> *_v, std::vector<ngl::Vec2> *_v0, float _a, float _c);

    void diffuse(int _b, std::vector<float> *_x, std::vector<float> *_x0);

    void diffuse();

    void advect(int _b, std::vector<float> *_d, std::vector<float> *_d0, std::vector<float> *_velocX, std::vector<float> *_velocY);

    void project(std::vector<float> *_velocX, std::vector<float> *_velocY, std::vector<float> *_p, std::vector<float> *_div);

    void splitter(std::vector<ngl::Vec2> _v, std::vector<float> *_x, std::vector<float> *_y)
    {
        _x->resize(_v.size());
        _y->resize(_v.size());

        for (size_t i = 0; i < _v.size(); i++)
        {
            _x->at(i) = _v[i].m_x;
            _y->at(i) = _v[i].m_y;
        }
    }

    void merger(std::vector<ngl::Vec2> *_v, std::vector<float> _x, std::vector<float> _y)
    {
        _v->resize(_x.size());

        for (size_t i = 0; i < _x.size(); i++)
        {
            _v->at(i) = ngl::Vec2{_x[i], _y[i]};
        }
    }
};

#endif // !FLUID_H_