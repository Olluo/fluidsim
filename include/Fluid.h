/**
 * @file Fluid.h
 * @brief This class implements the methods from https://www.dgp.toronto.edu/public_user/stam/reality/Research/pdf/GDC03.pdf.
 * They are slightly modified versions of the originals, trying to use C++ syntax instead of C
 * 
 * @copyright Copyright (c) 2021
 */

#ifndef FLUID_H_
#define FLUID_H_

#include <vector>

const int c_iter = 4;
const size_t c_size = 100;

class Fluid
{
public:
    /**
     * @brief Enum used to determine which boundary to set when set_boundary is called
     * 
     */
    enum class Boundary
    {
        None,
        X,
        Y
    };

    /**
     * @brief Set the boundaries of the fluid grid so that all exterior velocities are the inverse of the next layer 
     * inside the grid.
     * This stops the fluid from "leaking" out of the grid.
     */
    static void set_boundary(Boundary _b, std::vector<float> *_x);
    /**
     * @brief Diffuse the velocities through the grid by precalculating a value and using the linear_solve function to 
     * solve the partial differential equation
     */
    static void diffuse(Boundary _b, std::vector<float> *_x, std::vector<float> *_x0, float _diff, float _dt);
    /**
     * @brief Advect the velocities through the grid by going to the previous iteration and following the velocity 
     * backwards to find the affecting velocities, then calculates the weighted average and uses this new value.
     */
    static void advect(Boundary _b, std::vector<float> *_d, std::vector<float> *_d0, std::vector<float> *_velocX, std::vector<float> *_velocY, float _dt);
    /**
     * @brief Project the velocities making sure the fluid remains incompressible, fixing-up the data.
     */
    static void project(std::vector<float> *_velocX, std::vector<float> *_velocY, std::vector<float> *_p, std::vector<float> *_div);
    /**
     * @brief Converts XY coordinates into an index of a 1D array
     */
    static size_t IX(size_t _x, size_t _y)
    {
        return _x + _y * c_size;
    }
private:
    /**
     * @brief This class is static so don't allow construction
     */
    Fluid() {}

    /**
     * @brief Solve the partial differential equation using Gauss-Seidel reduction
     * 
     */
    static void linear_solve(Boundary _b, std::vector<float> *_x, std::vector<float> *_x0, float _a, float _c);
};

#endif // !FLUID_H_