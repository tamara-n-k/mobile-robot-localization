#pragma once

#include <vector>
#include <random>

class MCL
{
public:
    struct Particle
    {
        std::vector<double> state;  // [x, y, theta]
        double weight;
    };

    MCL(int num_particles,
        double x_min, double x_max,
        double y_min, double y_max);

    const std::vector<Particle>& particles() const;

private:
    int N_;
    std::vector<Particle> particles_;

    // map bounds
    double x_min_, x_max_;
    double y_min_, y_max_;

    // random number generation
    std::random_device rd_;
    std::mt19937 gen_;

    void initParticles();

};
