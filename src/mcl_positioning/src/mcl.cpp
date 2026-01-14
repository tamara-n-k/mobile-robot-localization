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
void MCL::motionUpdate(double dx, double dy, double dtheta)
{
    // Gaussian noise
    std::normal_distribution<double> noise_xy(0.0, 0.01);
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

void MCL::measurementUpdate(const std::vector<std::tuple<int, double, double>>& observations) {
    if (observations.empty()) return;

    const double sigma_range = 0.2;
    const double normalizer = 1.0 / (sigma_range * std::sqrt(2.0 * M_PI));

    for (auto& p : particles_) {
        double weight = 1.0;
        
        // particle orientation
        double cos_th = std::cos(p.state[2]);
        double sin_th = std::sin(p.state[2]);

        for (const auto& obs : observations) {
            int lm_id = std::get<0>(obs);
            double obs_y_local = std::get<2>(obs); 
            double obs_x_local = std::get<1>(obs); 
            
            auto lm_pos = landmark_manager_.getLandmark(lm_id);
            double lm_x_global = lm_pos.first;
            double lm_y_global = lm_pos.second;

            //obs converted to global frame
            double predicted_lm_x = p.state[0] + (obs_x_local * cos_th - obs_y_local * sin_th);
            double predicted_lm_y = p.state[1] + (obs_x_local * sin_th + obs_y_local * cos_th);

            double dx = lm_x_global - predicted_lm_x;
            double dy = lm_y_global - predicted_lm_y;
            double sq_dist = dx*dx + dy*dy;

            // gaussian likelihood model
            double prob = normalizer * std::exp(-0.5 * sq_dist / (sigma_range * sigma_range));
            weight *= (prob + 1e-9); 
        }
        p.weight = weight;
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
