#pragma once
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include <cmath>
#include "landmark_manager.hpp"

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

    // Getter für particles
    const std::vector<Particle>& getParticles() const;

    struct LandmarkMeasurement {
        int landmark_id;
        double range;  // observed distance to the landmark
    };

    void updateWeights(const std::vector<double>& laser_ranges,
                       const std::vector<double>& laser_angles);

    void motionModel(double dx, double dy, double dtheta);
    void normalizeWeights();
    void resample();

private:
    int N_;

    std::vector<Particle> particles_;

    // map bounds
    double x_min_, x_max_;
    double y_min_, y_max_;

    // random number generation
    std::random_device rd_;// produces a seed
    std::mt19937 gen_; // is initialized with that seed

    std::vector<std::vector<int>> map_;
    double map_resolution_;
    double map_origin_x_, map_origin_y_;

    void initParticles();

    double normalizeAngle(double angle);

    LandmarkManager landmark_manager_;
    void measurementUpdate(const std::vector<double>& laser_ranges,
                           const std::vector<double>& laser_angles);

    double sensorModel(const Particle& p,
                       const std::vector<double>& laser_ranges,
                       const std::vector<double>& laser_angles);

    double computeExpectedRange(const std::vector<double>& state, double beam_angle);

    void lowVarianceResample();
};
