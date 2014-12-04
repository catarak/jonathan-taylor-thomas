#Line-Following (and Box-Pick-up-ing) Robot

This was a project for a course I took in mechatronics in the Spring of 2012. As a group we created a line-following robot which had to successfully navigate a course and pick up two boxes along the way. 

All robot movements and actions were controlled using an Arduino. In order to follow a line, which was black tape on white paper, we implemented a PID controller with three light sensors on the front of the robot. As the robot navigated through the course, it would correct itself if it drifted away from the tape line. Driving the wheels of the robot were two DC Brush Motors, and controlling the arms of the robot were two servo motors. The arms picked up two magnetic boxes along navigating the course. 

Aside from writing the Arduino code (which is included in this project) to control the robot's actions, we also designed the circuits to control the robot (schematics included in the project's PDF writeup). 