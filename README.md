# 🤖 Pedestrian Assistance Robot in Webots

A simulated robotics project where a warden E-puck robot monitors and assists pedestrian E-puck robots crossing a zebra crossing using intelligent path planning and realistic movement behavior.

## 📽️ Demo
*Insert GIF or image of simulation here*

---

## 🛠️ Project Description

This project simulates a road-crossing scenario in Webots with three E-puck robots:

- **PEDESTRIAN1** and **PEDESTRIAN2**: Simulated pedestrians that cross a zebra crossing.
- **WARDEN**: A supervisory robot that monitors pedestrian robots. If a pedestrian gets stuck, the warden detects it and moves to assist using realistic rotation and translation movements.

### Key Features

- 🔍 **Supervisor Control**: The warden has access to global coordinates of other robots.
- 📍 **Path Planning**: Uses grid-based A* search for flexible path planning.
- 🚗 **Realistic Movement**: The warden rotates towards targets, then moves forward like a real robot.
- 🧠 **Assistance Logic**: Detects stuck pedestrians and navigates to them autonomously.

---

## 🧠 Technologies Used

- **Webots** (Robot simulator)
- **Python** (Robot controller and supervisor scripts)
- **A* Pathfinding** (For dynamic obstacle-free navigation)
- **E-puck Robot Model** (Simulated with differential drive)
  
---

## 📁 Project Structure

```plaintext
.
├── controllers/
│   ├── pedestrian_controller.py     # Logic for pedestrian movement
│   ├── warden_supervisor.py         # Supervisor logic + path planning
│
├── worlds/
│   └── zebra_crossing.wbt           # Webots world file
│
├── README.md
```

---

## 🚀 Getting Started

### Prerequisites

Before running the simulation, install the following:

- [Webots](https://cyberbotics.com/) (R2023b or later)  
- Python 3.7+  
- Required Python libraries:

```bash
pip install numpy

```


### Running the Simulation

Clone the Repository
Open a terminal and run:
```bash
git clone https://github.com/yourusername/epuck-warden.git
cd epuck-warden
```

Open the Project in Webots
- Launch Webots
- Open the world file:
		worlds/zebra_crossing.wbt

Assign Controllers to Robots
- Assign pedestrian_controller to PEDESTRIAN1 and PEDESTRIAN2
- Assign warden_supervisor to WARDEN


Run the Simulation
- Click the green Play button
- Watch the warden robot assist stuck pedestrians in real time


## 🔍 Future Improvements

- Replace static “stuck” logic with real IR sensor-based detection
- Add deep learning for real-time pedestrian recognition
- Improve navigation with advanced path planners (e.g., D* Lite, RRT)
- Introduce smart traffic light control and vehicle avoidance