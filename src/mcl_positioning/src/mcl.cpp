#include "mcl.hpp"
#include <cmath>

// constructor
MCL::MCL(int num_particles,
         double x_min, double x_max,
         double y_min, double y_max)
    : N_(num_particles),
      x_min_(x_min), x_max_(x_max),
      y_min_(y_min), y_max_(y_max),
      gen_(rd_())
{
    initParticles();
}

// particle init
void MCL::initParticles()
{
    std::uniform_real_distribution<double> dist_x(x_min_, x_max_);
    std::uniform_real_distribution<double> dist_y(y_min_, y_max_);
    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

    particles_.resize(N_);

    const double uniform_weight = 1.0 / static_cast<double>(N_);

    for (auto &p : particles_)
    {
        p.state.resize(3);
        p.state[0] = dist_x(gen_);      // x
        p.state[1] = dist_y(gen_);      // y
        p.state[2] = dist_theta(gen_);  // theta
        p.weight = uniform_weight;
    }
}


