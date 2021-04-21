/**
 * @file Fluid.cpp
 * @brief This class implements the methods from https://www.dgp.toronto.edu/public_user/stam/reality/Research/pdf/GDC03.pdf.
 * They are slightly modified versions of the originals, trying to use C++ syntax instead of C
 * 
 * @copyright Copyright (c) 2021
 */

#include "Fluid.h"

void Fluid::set_boundary(Boundary _b, std::vector<float> *_x)
{
    for (int i = 1; i < c_size - 1; i++)
    {
        _x->at(IX(i, 0)) = _b == Boundary::Y ? -_x->at(IX(i, 1)) : _x->at(IX(i, 1));
        _x->at(IX(i, c_size - 1)) = _b == Boundary::Y ? -_x->at(IX(i, c_size - 2)) : _x->at(IX(i, c_size - 2));
    }
    for (int j = 1; j < c_size - 1; j++)
    {
        _x->at(IX(0, j)) = _b == Boundary::X ? -_x->at(IX(1, j)) : _x->at(IX(1, j));
        _x->at(IX(c_size - 1, j)) = _b == Boundary::X ? -_x->at(IX(c_size - 2, j)) : _x->at(IX(c_size - 2, j));
    }

    _x->at(IX(0, 0)) = 0.5f * (_x->at(IX(1, 0)) + _x->at(IX(0, 1)));
    _x->at(IX(0, c_size - 1)) = 0.5f * (_x->at(IX(1, c_size - 1)) + _x->at(IX(0, c_size - 2)));
    _x->at(IX(c_size - 1, 0)) = 0.5f * (_x->at(IX(c_size - 2, 0)) + _x->at(IX(c_size - 1, 1)));
    _x->at(IX(c_size - 1, c_size - 1)) = 0.5f * (_x->at(IX(c_size - 2, c_size - 1)) + _x->at(IX(c_size - 1, c_size - 2)));
}

void Fluid::linear_solve(Boundary _b, std::vector<float> *_x, std::vector<float> *_x0, float _a, float _c)
{
    float cRecip = 1.0f / _c;
    for (int k = 0; k < c_iter; k++)
    {
        for (int j = 1; j < c_size - 1; j++)
        {
            for (int i = 1; i < c_size - 1; i++)
            {
                _x->at(IX(i, j)) =
                    (_x0->at(IX(i, j)) + _a * (_x->at(IX(i + 1, j)) + _x->at(IX(i - 1, j)) + _x->at(IX(i, j + 1)) + _x->at(IX(i, j - 1)))) * cRecip;
            }
        }

        set_boundary(_b, _x);
    }
}

void Fluid::diffuse(Boundary _b, std::vector<float> *_x, std::vector<float> *_x0, float _diff, float _dt)
{
    float a = _dt * _diff * (c_size - 2) * (c_size - 2);
    linear_solve(_b, _x, _x0, a, 1 + 4 * a);
}

void Fluid::advect(Boundary _b, std::vector<float> *_d, std::vector<float> *_d0, std::vector<float> *_velocX, std::vector<float> *_velocY, float _dt)
{
    float i0, i1, j0, j1;

    float dtx = _dt * (c_size - 2);
    float dty = _dt * (c_size - 2);

    float s0, s1, t0, t1;
    float tmp1, tmp2, x, y;

    float Nfloat = static_cast<float>(c_size);
    int i, j;
    float ifloat, jfloat;

    for (j = 1, jfloat = 1.0f; j < c_size - 1; j++, jfloat++)
    {
        for (i = 1, ifloat = 1.0f; i < c_size - 1; i++, ifloat++)
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

    set_boundary(_b, _d);
}

void Fluid::project(std::vector<float> *_velocX, std::vector<float> *_velocY, std::vector<float> *_p, std::vector<float> *_div)
{

    for (int j = 1; j < c_size - 1; j++)
    {
        for (int i = 1; i < c_size - 1; i++)
        {
            _div->at(IX(i, j)) = -0.5f * (_velocX->at(IX(i + 1, j)) - _velocX->at(IX(i - 1, j)) + _velocY->at(IX(i, j + 1)) - _velocY->at(IX(i, j - 1))) / c_size;
            _p->at(IX(i, j)) = 0;
        }
    }

    set_boundary(Boundary::None, _div);
    set_boundary(Boundary::None, _p);
    linear_solve(Boundary::None, _p, _div, 1, 6);

    for (int j = 1; j < c_size - 1; j++)
    {
        for (int i = 1; i < c_size - 1; i++)
        {
            _velocX->at(IX(i, j)) -= 0.5f * (_p->at(IX(i + 1, j)) - _p->at(IX(i - 1, j))) * c_size;
            _velocY->at(IX(i, j)) -= 0.5f * (_p->at(IX(i, j + 1)) - _p->at(IX(i, j - 1))) * c_size;
        }
    }
    set_boundary(Boundary::X, _velocX);
    set_boundary(Boundary::Y, _velocY);
}