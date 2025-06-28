# Technical Documentation

## Introduction

### Team
* Coach: 
    * **Engr Dr Balogun Wasiu Adebayo**: A senior lecturer at the Lagos State University of Science and Technology whose research interests lie at the intersection of Mechatronics, Artificial Intelligence, and Automation, focusing on enhancing distributed systems control, precision engineering, and automotive electronics. He is particularly interested in developing intelligent grading systems, optimizing robust control mechanisms, and applying machine vision in automotive applications. His work also explores advancements in robust decentralized control, leveraging optimization techniques.
* Team Members: 
    * **Ohieku Eneji Peacemaker**: A 300L engineering student from the Lagos State University of Science and Technology, Department Of Mechatronics Engineering. A very passionate problem solver involving in activities such as Competive Programming (Math/Coding Sports), Embedded Systems Engineering & Robotics, CAD modeling (Mechanical and Electronics Designs), Mathematical Problem solving and thinking and Machine Learning/Deep Learning. He has participated and won Hackathons, assisted researches, Built Robots, Embedded Systems and Programs.

    * **Agbede Collins Theophilus**: He is a 300-level Electrical and Electronics Engineering student at Lagos State University of Science and Technology with a strong interest in mathematics and robotics. He is passionate about applying mathematical principles to solve real-world engineering problems, particularly in control systems, automation, and intelligent hardware design. Collins is actively exploring robotics projects and sharpening his skills in areas such as kinematics, circuit design, and algorithm development. His goal is to contribute to innovative solutions at the intersection of engineering, mathematics, and technology.

### Project Overview
In the light of profering a solution to the Self driving Car Challenge FE 2025 we proposed the development of a virtual robotic simulation environment. This has a virtual robot with kinematic properties as an ackermann steer drive (As described in the FE Rules Document), The Mat also according exact dimension specified, Randomness such as coin tosses, card selection/shuffling and roll of a die. All in simulation helping us test ideas and control software in a virtual environment.

***

## Core Technology Used
We will be dicussing the technologies we in the development of this project.
* **Robot Operating System (ROS)**: A powerful set of software libraries and tools for building robot applications, providing drivers, algorithms, and developer tools for robotics projects. Usually run on ubuntu ros aids the development of robots.
* **Gazebo Simulation Software**: Gazebo brings a fresh approach to simulation with a complete toolbox of development libraries and cloud services to make simulation easy. Iterate fast on your new physical designs in realistic environments with high fidelity sensors streams. Test control strategies in safety, and take advantage of simulation in continuous integration tests.
* **Unified Robot Description Format (URDF)**: it is an XML format for representing a robot model or physical structures.URDF is commonly used in Robot Operating System (ROS) tools such as rviz (Ros Visualization tool) and Gazebo simulator. The model consists of links and joints motion.
* **Xacro (XML Macro)**: is a macro language for XML documents, often used in robotics and simulation environments like ROS (Robot Operating System). It simplifies complex XML files by allowing users to define reusable macros, generate robot descriptions, and configurations. Xacro files are typically converted to URDF (Unified Robot Description Format) or other formats for use in simulations or robots.
* **OpenCV (Open Source Computer Vision Library)**: is a widely-used library for computer vision and image processing, providing tools for tasks like image and video processing, object detection, facial recognition, tracking, and machine learning. It's commonly used in applications such as robotics, surveillance, self-driving cars, medical imaging, and augmented reality, with its open-source nature making it accessible and customizable.


***

## System Architecture
In this section we will be describing the entire system by breaking them down into small understable units, giving an overview and the relationship between those units.
* **Robot Model Description**: The robot model was created by using URDF to describe kinematic Structure. Each parts of the robot is describe as a link having visual, collision and inertial properties. Every link has a relationship to another link in a tree structure called joints which were either fixed, revolute, continuous, prismatic etc. ![robot model]()


