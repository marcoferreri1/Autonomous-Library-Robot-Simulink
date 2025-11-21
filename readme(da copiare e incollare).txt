# Simulink-Based Design and Control of an Autonomous Library Assistant Robot



![MIT License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

![Maintained](https://img.shields.io/badge/status-maintained-brightgreen?style=for-the-badge)

![MATLAB](https://img.shields.io/badge/MATLAB-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)

![Simulink](https://img.shields.io/badge/Simulink-orange?style=for-the-badge&logo=mathworks&logoColor=white)



**Digital Twin simulation of an autonomous differential drive robot for library navigation. Features PRM path planning in MATLAB and double-loop PI control in Simulink.**



---



## 📋 Prerequisites



To run this simulation, ensure you have the following environment:



* **Knowledge:**

    * Fundamentals of Robotics (Differential Drive Kinematics)

    * Control Theory (PI Controllers)

    * MATLAB/Simulink environment usage

* **Hardware:**

    * PC with Windows, Linux, or macOS capable of running MATLAB/Simulink.

* **Software:**

    * MATLAB (Recommended: R2023b or newer)

    * Simulink

    * **Required Toolboxes:**

        * Robotics System Toolbox

        * Navigation Toolbox

        * Control System Toolbox



---



## 📖 Introduction



Large libraries face operational challenges in guiding users and managing book logistics. Autonomous robots offer a solution to improve efficiency. This project addresses the need for a robust navigation system, proposing a design validated entirely through simulation.



We present a complete framework, from path planning in a virtual map to high-precision motor control, demonstrating a viable proof-of-concept ("Digital Twin") for a library assistant robot.



**Project Objectives:**

* Develop a virtual library environment using Matlab's `binaryOccupancyMap`.

* Implement a Probabilistic Roadmap (PRM) algorithm for global, collision-free path planning.

* Generate smooth, time-based trajectories using cubic polynomial interpolation.

* Design a double-loop (cascade) control system in Simulink for precise trajectory tracking.

* Validate the system's performance by comparing desired vs. actual simulated paths.



---



## 🔧 Simulation Environment



**Description of the environment used:**



* **Main Simulator:** MATLAB & Simulink

* **Map Generation:** `binaryOccupancyMap` (inflated by robot radius)

* **Path Planning Algorithm:** `mobileRobotPRM` (Probabilistic Roadmap)

* **Control Architecture:** Simulink Block Diagram (Cascade Control)



---



## 💾 Installation



Follow these steps to set up the simulation environment:



1.  **Clone the repository:**

    ```bash

    git clone [https://github.com/marcoferreri/Autonomous-Library-Robot-Simulink.git](https://github.com/marcoferreri/Autonomous-Library-Robot-Simulink.git)

    ```

2.  **Open MATLAB:**

    Navigate to the cloned folder within MATLAB.

3.  **Add to Path:**

    Right-click the folder in MATLAB's "Current Folder" window and select *Add to Path -> Selected Folders and Subfolders*.



---


## 🛠️ Configuration & Usage

Instructions to run the simulation:

1.  **Open the Main Script:**
    Open `main_run_simulation.m` in MATLAB.

2.  **Run the Script:**
    Click the **Run** button (or press F5).

    *Note:* You do **not** need to open the Simulink model manually. This master script automatically:
    * Calls `defineSimscapeParameters.m` and `planLibraryPath.m` to initialize parameters and map.
    * Executes `generateTrajectoryAndRunSim.m`.
    * Opens and runs the Simulink model `diff_drive.slx`.

3.  **Visualize:**
    Once the script finishes, the simulation results and trajectory plots will be displayed automatically.


---



## 💻 Code Example / Programming

The core of the navigation logic relies on the **Probabilistic Roadmap (PRM)** algorithm.

The following snippet demonstrates how the map is prepared and the planner is configured. First, the map is **inflated** by the robot's radius to create a safety margin (ensuring the physical robot doesn't scrape against obstacles). Then, the PRM is initialized with a high density of nodes to find a valid path through the library aisles.

```matlab
% --- 2. ROBOT PREPARATION & MAP INFLATION ---
robotRadius = 1.0; % Robot radius for path planning (m)
mapInflated = copy(libraryMap);
inflate(mapInflated, robotRadius);

% --- 3. PATH PLANNER (PRM) CONFIGURATION ---
prm = mobileRobotPRM;
prm.Map = mapInflated;
prm.NumNodes = 2500;         % Number of random points scattered on the map
prm.ConnectionDistance = 10; % Max distance to connect two nodes

```


---


## ✅ Conclusion



All project objectives were successfully met. We have designed and validated a complete autonomous navigation system for a library robot.



**Key Findings:**

* The combination of **PRM planning** and a **double-loop PI/PI controller** provides a robust and highly accurate solution.

* Simulation results show minimal tracking error for X, Y, and Theta variables.

* This simulation serves as a validated "digital twin," significantly reducing the risk and cost of physical prototyping.


---


## 🔜 Future Improvements


* **Physical Prototype:** Construct a real-world robot based on the parameters validated in this simulation.

* **Dynamic Obstacles:** Implement local path planning (e.g., VFH algorithm) to avoid moving people or objects in real-time.

* **Sensor Integration:** Simulate LiDAR or Camera inputs for SLAM (Simultaneous Localization and Mapping) to replace the ideal map knowledge.


---


## ⚠️ Disclaimer


As indicated in the MIT License, this software is provided **"as is", without warranty of any kind**. The authors are not responsible for any damage to hardware, loss of data, or other issues resulting from the use of this code.


---


## 📚 Additional Resources



* [MathWorks: Mobile Robot Algorithms (Robotics System Toolbox)](https://www.mathworks.com/help/robotics/mobile-robot-algorithms.html)

* [Probabilistic Roadmaps (Kavraki et al., 1996)](https://doi.org/10.1109/70.508439)

* [Springer Handbook of Robotics](https://link.springer.com/book/10.1007/978-3-319-32552-1)


---



## 👥 Authors


* **Marco Ferreri** - *Project Lead**

* **Pietro Borracelli** - *Project Lead**

* **Hugo Valdés Ortega**
* **José Francisco Reyes Maldonado**
* **Emilio Rafael García García**
* **Paul Pfister**
* **Irving Alejandro Vásquez Salinas**
* **Irving Alejandro Vásquez Salinas**
* **Irving Alejandro Vásquez Salinas**



---


## 📬 Contact


This project is actively **maintained by Marco Ferreri and Pietro Borracelli**.

If you require assistance with the simulation, have technical questions regarding the control architecture, or wish to propose improvements, please do not hesitate to reach out directly to the maintainer.

* **Email:** marcoferreri.p@gmail.com
* **Email:** pietro.borracelli@gmail.com

For bug reports or specific feature requests, you are also encouraged to open a new **Issue** in this repository.