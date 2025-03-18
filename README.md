# 6-DOF Cobot Simulation: Kinematics, Trajectory Planning, and Workspace Visualization

This repository presents a **6-DOF Collaborative Robot (Cobot) Simulation** demonstrating *forward kinematics*, *inverse kinematics*, *trajectory planning*, and *operational workspace* analysis. Developed with **MATLAB** and the **Peter Corke Robotics Toolbox**, the project is suitable for **PhD-level research** and **hands-on demonstrations**—covering everything from DH-based modeling to multi-point trajectory generation.

## Table of Contents
1. **Key Features**  
2. **Technical Overview (PhD-Level)**  
3. **Repository Structure**  
4. **Step-by-Step Setup**  
5. **Execution & Working Explanation**  
6. **Future Extensions**  
7. **Contact & License**

## 1. Key Features
- **Forward & Inverse Kinematics**  
  - Calculate end-effector poses from joint angles (FK).  
  - Solve for joint angles to reach a given [x, y, z] target (IK).

- **Trajectory Planning**  
  - Generate smooth joint-space paths between user-defined points (A, B, C).  
  - Visualize the resulting Cartesian motion and joint configurations.

- **Operational Workspace Visualization**  
  - Systematically sample joint angles, compute forward kinematics, and plot the reachable region in 3D.

- **Jacobian & Dynamics**  
  - Compute analytical/numerical Jacobians.  
  - Observe velocities and accelerations along multi-segment trajectories for dynamic analysis.

## 2. Technical Overview (PhD-Level)
1. **Kinematic Modeling**  
   - **Forward Kinematics (FK):** Denavit–Hartenberg (DH) parameters define each link’s transform. The final homogeneous transform yields the end-effector pose.  
   - **Inverse Kinematics (IK):** Numerical solvers (e.g., `ikine`) in Peter Corke’s Toolbox find joint angles for target positions; orientation is optionally constrained.

2. **Trajectory Generation**  
   - **Joint-Space Interpolation:** `jtraj` produces polynomial-based joint paths with continuous velocity and acceleration. This ensures smooth manipulator motion.

3. **Operational Workspace**  
   - Nested loops vary each joint across a given range (±π or ±π/2). Forward kinematics compute the corresponding end-effector positions. The resulting point cloud reveals the manipulator’s reachable volume.

4. **Jacobian Computation & Manipulability**  
   - The Jacobian (`robot.jacob0(q)`) connects joint velocities with end-effector velocities. Analyzing it helps identify singularities, measure manipulability, and develop advanced controllers.

5. **Velocity & Acceleration Profiles**  
   - By sampling the trajectory (e.g., from points A→B→C), the code extracts per-joint velocities and accelerations over time. This is critical for ensuring dynamic feasibility in real-world applications.

## 3. Repository Structure
- **MAE547-CobotSimulation.m**  
  - Main MATLAB script for:
    - Forward & Inverse Kinematics  
    - Trajectory Planning (A→B→C)  
    - Workspace Sampling & Plotting  
    - Jacobian & Joint Vel/Accel Analysis
- **Readme.txt**  
  - Additional instructions and licensing info.

## 4. Step-by-Step Setup
1. **Clone or Download**
   ```bash
   https://github.com/Ashish-Paka/6DOFCOBOT_TrajectoryPlanning_Kinematics.git
   cd Code/6DOF-Cobot-Simulation
   ```

2. **Install MATLAB & Toolboxes**
   - [MATLAB](https://www.mathworks.com/products/matlab.html) (R2020b or later recommended).  
   - [Peter Corke’s Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/).

3. **Add Folder to MATLAB Path**
   - In MATLAB, go to **Set Path** → **Add Folder** → select `6DOF-Cobot-Simulation`.  
   - Save the path changes.

4. **Confirm Installations**
   - Open MATLAB, type `ver` or check Add-On Explorer to ensure required toolboxes are present.

## 5. Execution & Working Explanation
1. **Launch MATLAB**
   - Navigate to the `6DOF-Cobot-Simulation` folder.

2. **Open & Run Script**
   - Open `MAE547-CobotSimulation.m`.
   - Click **Run** or type:
     ```matlab
     run('MAE547-CobotSimulation.m')
     ```

3. **Define Points A, B, C**
   - The script prompts for three Cartesian coordinates (e.g., `[0.5, 0.2, 0.3]`).  
   - Input feasible points within reach; if IK fails (`NaN`), select alternatives.

4. **Kinematics & Trajectory**
   - Inverse kinematics finds joint angles (`qA`, `qB`, `qC`) for the given points.  
   - `jtraj` then generates smooth paths from A→B and B→C.

5. **Visualization**
   - A 3D figure displays the manipulator moving along the path with scattered markers.  
   - Additional plots show:
     - The operational workspace (3D scatter from sampling the joint space).  
     - Joint velocity and acceleration curves over the trajectory.

6. **Jacobian & Dynamic Profiles**
   - The script computes the Jacobian at a selected configuration, demonstrating how local velocities map to the end-effector frame.  
   - Velocity/acceleration subplots provide insight into kinematic feasibility and safety constraints.

## 6. Future Extensions
- **Time-Optimal Trajectories & Constraint Handling**
  - Incorporate jerk/torque limits and polynomial or spline-based time parameterization.
- **Full Dynamic Modeling**
  - Employ Newton-Euler or Lagrangian equations for torque calculations, energy usage, and payload constraints.
- **ROS/ROS2 Integration**
  - Bridge MATLAB with real or simulated cobots for hardware-in-the-loop testing.
- **Advanced Motion Planning**
  - Add obstacle avoidance using PRM, RRT*, or other sampling-based planners.

## 7. Contact & License
**Authors & Inquiries**
- **Manaswi Thinagaran** — mthinaga@asu.edu  
- **Ashish Paka** — apaka@asu.edu  
- **Akshay Sopan Mahall** — amahalle@asu.edu

**Licensing**
- MATLAB code subject to [MathWorks Terms](https://www.mathworks.com/help/matlab/matlab_external/mathworks-terms-of-service.html).  
- Peter Corke’s Toolbox under [GNU LGPL v3](https://www.gnu.org/licenses/lgpl-3.0.html).  
- Refer to `Readme.txt` for additional details.

**Enjoy exploring 6-DOF kinematics & trajectory planning!**  
Feel free to open issues or submit pull requests for further collaboration.
