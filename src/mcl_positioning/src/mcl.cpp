#include "mcl_positioning/mcl.hpp"

// constructor
MCL::MCL(int num_particles,
         double x_min, double x_max,
         double y_min, double y_max,
         const LandmarkManager& landmark_manager):
        N_(num_particles),
        x_min_(x_min), x_max_(x_max),
        y_min_(y_min), y_max_(y_max),
        landmark_manager_(landmark_manager),
        gen_(rd_())

{
    initParticles();
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
void MCL::motionUpdate(double dx, double dy, double dtheta, double dev_xy, double dev_theta)
{
    // Gaussian noise
    std::normal_distribution<double> noise_xy(0.0, dev_xy);
    std::normal_distribution<double> noise_theta(0.0, dev_theta);
    
    for (auto &p : particles_)//for every particle, update motion
        {
            // odometry + noise (draw 1 unique random sample from gaussian distr. using gen_)
            p.state[0] += dx + noise_xy(gen_);
            p.state[1] += dy + noise_xy(gen_);
            p.state[2] += dtheta + noise_theta(gen_);
            p.state[2] = normalizeAngle(p.state[2]);
        }
    }
void MCL::measurementUpdate(
    const std::vector<std::tuple<int, double, double>>& observations)
{
    if (observations.empty()) return;

    std::vector<double> log_weights(N_);
    
    for (size_t i = 0; i < particles_.size(); ++i) {
        auto& p = particles_[i];
        double log_weight = 0.0;

        double px = p.state[0];
        double py = p.state[1];
        double ptheta = p.state[2];

        double c = std::cos(ptheta);
        double s = std::sin(ptheta);

        for (const auto& obs : observations) {
            int lm_id = std::get<0>(obs);
            double obs_x = std::get<1>(obs);
            double obs_y = std::get<2>(obs);

            if (!landmark_manager_.hasLandmark(lm_id)) {
                continue;
            }

            auto lm_pos = landmark_manager_.getLandmark(lm_id);
            double lm_x = lm_pos.first;
            double lm_y = lm_pos.second;
            
            // distance from landmark to particle
            double dx_world = lm_x - px;
            double dy_world = lm_y - py;
            
            // transform into particle body frame
            double expected_x =  c * dx_world + s * dy_world;
            double expected_y = -s * dx_world + c * dy_world;
            
            // compare expected distance to observed distance in particle body frame
            double error_x = expected_x - obs_x;
            double error_y = expected_y - obs_y;
            double distance = std::sqrt(error_x * error_x + error_y * error_y);

            // likelihood model
            double prob_hit  = Z_HIT * comp_gaussian_prob(
                                   distance, 0.0, measurement_noise_);
            double prob_rand = Z_RAND / Z_MAX;
            double prob = prob_hit + prob_rand;
            
            log_weight += std::log(prob + 1e-100);
        }

        log_weights[i] = log_weight;
    }

    double max_log_weight = *std::max_element(log_weights.begin(), log_weights.end());
    
    for (size_t i = 0; i < particles_.size(); ++i) {
        particles_[i].weight = std::exp(log_weights[i] - max_log_weight);
    }

    normalizeWeights();
}

//A4. resampling
void MCL::resample()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(N_);

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

double MCL::comp_gaussian_prob(double value, double mean, double stddev) {
    double coeff = 1.0 / (stddev * sqrt(2.0 * M_PI));
    double exponent = -0.5 * std::pow((value - mean) / stddev, 2);
    return coeff * exp(exponent);
}
