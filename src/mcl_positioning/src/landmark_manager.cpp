#include "kalman_positioning/landmark_manager.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

/**
 * STUDENT ASSIGNMENT: Landmark Manager Implementation
 * 
 * This class manages landmark positions loaded from a CSV file.
 * Students should implement the methods for loading, querying, and 
 * managing landmark data.
 */

// ============================================================================
// CSV FILE LOADING
// ============================================================================

/**
 * @brief Load landmarks from CSV file
 * 
 * STUDENT TODO:
 * 1. Open the CSV file at csv_path
 * 2. Parse each line as: id,x,y
 * 3. Handle comments (lines starting with #)
 * 4. Store landmark positions in the landmarks_ map
 * 5. Return true if successful, false otherwise
 */
bool LandmarkManager::loadFromCSV(const std::string& csv_path) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open landmark file: " << csv_path << std::endl;
        return false;
    }
    
    landmarks_.clear();
    std::string line;
    int line_num = 0;
    
    while (std::getline(file, line)) {
        ++line_num;
        
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        std::string token;
        int id;
        double x, y;

        // Parse ID
        if (!std::getline(ss, token, ',')) {
            std::cerr << "Error parsing ID at line " << line_num << std::endl;
            continue;
        }
        try {
            id = std::stoi(token);
        } catch (const std::exception& e) {
            std::cerr << "Invalid ID at line " << line_num << ": " << token << std::endl;
            continue;
        }

        // Parse x
        if (!std::getline(ss, token, ',')) {
            std::cerr << "Error parsing x at line " << line_num << std::endl;
            continue;
        }
        try {
            x = std::stod(token);
        } catch (const std::exception& e) {
            std::cerr << "Invalid x at line " << line_num << ": " << token << std::endl;
            continue;
        }

        // Parse y
        if (!std::getline(ss, token, ',')) {
            std::cerr << "Error parsing y at line " << line_num << std::endl;
            continue;
        }
        try {
            y = std::stod(token);
        } catch (const std::exception& e) {
            std::cerr << "Invalid y at line " << line_num << ": " << token << std::endl;
            continue;
        }

        landmarks_[id] = std::make_pair(x, y);
    }
    
    file.close();
    
    if (landmarks_.empty()) {
        std::cerr << "Warning: No landmarks loaded from " << csv_path << std::endl;
        return false;
    }
    
    std::cout << "Loaded " << landmarks_.size() << " landmarks from " << csv_path << std::endl;
    return true;
}
// ============================================================================
// LANDMARK QUERIES
// ============================================================================

/**
 * @brief Get all landmarks
 */
const std::map<int, std::pair<double, double>>& LandmarkManager::getLandmarks() const {
    return landmarks_;
}

/**
 * @brief Get landmark position by ID
 */
std::pair<double, double> LandmarkManager::getLandmark(int id) const {
    auto it = landmarks_.find(id);
    if (it != landmarks_.end()) {
        return it->second;
    }
    return {0.0, 0.0};
}

/**
 * @brief Check if landmark exists
 */
bool LandmarkManager::hasLandmark(int id) const {
    return landmarks_.find(id) != landmarks_.end();
}

// ============================================================================
// SPATIAL QUERIES
// ============================================================================

/**
 * @brief Get landmarks within a certain radius of a point
 * 
 * STUDENT TODO:
 * 1. Iterate through all landmarks
 * 2. Calculate distance from (x, y) to each landmark
 * 3. Add landmarks within radius to result vector
 * 4. Return the vector of landmark IDs
 */
std::vector<int> LandmarkManager::getLandmarksInRadius(double x, double y, double radius) const {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    std::vector<int> result;
    for (const auto& [id, pos] : landmarks_) {
        if (distance(x, y, pos.first, pos.second) <= radius) {
            result.push_back(id);
        }
    }
    return result;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Calculate Euclidean distance between two points
 */
double LandmarkManager::distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

Eigen::Vector2d LandmarkManager::measurementModel(const std::vector<double>& state, int landmark_id) const
{
    if (!hasLandmark(landmark_id)) {
        return Eigen::Vector2d::Zero();
    }

    auto [lx, ly] = getLandmark(landmark_id);

    Eigen::Vector2d rel_pos(lx - state[0], ly - state[1]);
    double theta = state[2];

    Eigen::Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);

    // Transform landmark position into robot frame
    Eigen::Vector2d measurement = R.transpose() * rel_pos;

    return measurement;
}