source /opt/
colcon build
source install/setup.bash
ros2 launch kalman_positioning positioning.launch.py
ros2 launch fake_robot fake_robot.launch.py

rm -rf build/ install/ log/


A. Source Code: All modified CPP files (ukf.cpp, landmark_manager.cpp)
B1 & B2: Experiment Reports: PDF files for Part B with:
Configuration tables
Result diagrams
Quantitative measurements
Qualitative interpretations


Task B3: Simulation Parameter Variation (10 Points)
Test 5 scenarios: (1) Baseline, (2) Large curve radius, (3) High sensor noise, (4) Small observation radius, (5) Large observation radius:

Analyze impact of curve radius, sensor noise, and observation radius
Create table and plots comparing performance across scenarios

task B pipeline: 5 simulations:
1. start rosbag to collect data
2. create python script that computes rmse, convergence speed and stability
3. add rmse and corresponding process noise to csv & generate plot
4. document best  process noise parameter and explain behaviour



3. Reflection: 1-2 page summary
What did I learn?
What challenges were there?
How would I proceed if I started again?