#include "mcl_positioning/mcl.hpp"

// constructor
MCL::MCL(int num_particles,
         double x_min, double x_max,
         double y_min, double y_max)
    : N_(num_particles),
      x_min_(x_min), x_max_(x_max),
      y_min_(y_min), y_max_(y_max),
      gen_(rd_())//random number engine
{
    initParticles();
    if (!landmark_manager_.loadFromCSV("path_to_landmarks.csv")) {
    std::cerr << "Failed to load landmarks\n";
}

}
const std::vector<MCL::Particle>& MCL::getParticles() const {
    return particles_;
}

// A1. particle init
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

//A2. apply odometry based motion model to each particle
void MCL::motionModel(double dx, double dy, double dtheta)
{
    // Gaussian noise
    std::normal_distribution<double> noise_xy(0.0, 0.02);
    std::normal_distribution<double> noise_theta(0.0, 0.01);

    for (auto &p : particles_)//for every particle, update motion
        {
            // odometry + noise (draw 1 unique random sample from gaussian distr. using gen_)
            p.state[0] += dx + noise_xy(gen_);
            p.state[1] += dy + noise_xy(gen_);
            p.state[2] += dtheta + noise_theta(gen_);
            p.state[2] = normalizeAngle(p.state[2]);
        }
    }

// //A3. compute likelihood of each particle using laser scan data and map
// double MCL::sensorModel(const Particle& p,
//                         const std::vector<LandmarkMeasurement>& measurements)
// {
//     double q = 1.0;
//     const double sigma_range = 0.2; // example noise stddev in meters

//     for (const auto& meas : measurements)
//     {
//         // Expected measurement vector (relative landmark position in sensor frame)
//         Eigen::Vector2d expected_meas = landmark_manager_.measurementModel(p.state, meas.landmark_id);

//         // Extract expected range (distance)
//         double expected_range = expected_meas.norm();

//         // Actual measured range from CSV
//         double observed_range = meas.range;

//         // Gaussian likelihood on range difference
//         double diff = observed_range - expected_range;
//         double prob = (1.0 / (sigma_range * sqrt(2 * M_PI))) * std::exp(-0.5 * diff * diff / (sigma_range * sigma_range));

//         q *= prob;
//     }
//     return q;
// }

//A4. resampling
void MCL::resample()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(N_);

    // Low Variance Sampler algorithm
    std::uniform_real_distribution<double> dist_start(0.0, 1.0 / N_);
    double r = dist_start(gen_);
    double c = particles_[0].weight;
    int i = 0;

    for (int m = 0; m < N_; ++m)
    {
        double u = r + (double)m / N_;
        while (u > c)
        {
            i++;
            c += particles_[i].weight;
        }
        new_particles.push_back(particles_[i]);
        // Reset weight to uniform for the next iteration
        new_particles.back().weight = 1.0 / N_;
    }

    particles_ = std::move(new_particles);
}      
        

//helpers
double MCL::normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

void MCL::normalizeWeights()
{
    double sum_weights = 0.0;
    for (const auto& p : particles_)
    {
        sum_weights += p.weight;
    }

    if (sum_weights < 1e-12)//too small 
    {   
        // std::cout << " Sum of weights is too small." << std::endl;
        double uniform_weight = 1.0 / static_cast<double>(N_);
        for (auto& p : particles_)
        {
            p.weight = uniform_weight;
        }
        return;
    }

    for (auto& p : particles_)
    {
        p.weight /= sum_weights;
    }
}
