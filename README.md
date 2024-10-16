Overview:

This project involves building an autonomous smart car using the ESP32 microcontroller. The car is equipped with multiple sensors including radar, LiDAR, and a camera for environmental perception, obstacle detection, and real-time path planning. It uses the A* search algorithm for dynamic pathfinding and constructs a map based on sensor inputs to navigate through its environment efficiently. The system also integrates PWM motor control to precisely regulate speed and direction.

Features:

Radar, LiDAR, and Camera Integration: Utilizes radar for long-range obstacle detection, LiDAR for high-precision environmental mapping, and a camera for visual feedback.
A* Path Planning Algorithm: Implements the A* algorithm to find the optimal path, dynamically updating as new obstacles are detected.
Dynamic Map Construction: Continuously builds and updates a map of the environment using sensor data for real-time navigation.
PWM Motor Control: Regulates motor speed and direction using PWM signals based on path planning results.
Bluetooth Communication: Includes Bluetooth capability to allow remote monitoring and control via a custom GUI.

Hardware Components:
ESP32 Microcontroller: The brain of the car, handling sensor data, control logic, and communication.
Motors: Four motors controlled by PWM to enable precise movement.
Sensors:
Radar: Detects distant obstacles.
LiDAR: Provides accurate distance measurements for mapping.
Camera: Assists with visual navigation and obstacle detection.
Power Supply: A battery unit powers the ESP32 and motors.

Software Overview:
Path Planning: The system uses the A* algorithm to calculate the shortest path, updating dynamically with real-time obstacle detection.
PWM Motor Control: Motor speed and direction are controlled using PWM signals, calculated based on the current path and sensor inputs.
Bluetooth Interface: The ESP32 uses Bluetooth for remote communication, allowing real-time data monitoring and manual overrides.
