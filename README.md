This project was done as a part of research internship with Dr.Guillaume , NUS Singapore.

The main task was creating a controller for a swarm of drones and deconflict setpoints and simulate the following in ROS and Gazebo.

The main task was to specifically use crazyflie drones which was difficult to control with standard controllers like mellinger controller etc. For the simulation task, urdf models etc G.Silano's CrazyS repo was used.

To overcome this a sliding mode control was tested and a square trajectory was to be followed. Works's still in progress to make it more robust and follow the path smoothly.

The following results show the working of a single drone using sliding mode control. Since it gave imporved results we moved on to test for multiple drones and tried to make a trajectory generating algorithm like splone trajectory generator 


[Screencast from 27-07-24 01:49:15 PM IST.webm](https://github.com/user-attachments/assets/c2a35da8-70e0-42a7-8363-77074f7f4153)

[Screencast from 27-07-24 01:47:36 PM IST.webm](https://github.com/user-attachments/assets/81365104-6162-4c6c-85ab-a7461621de01)


Collison avoidance is also a critical issue when it comes to swarm robotics. For this purpose artificial potential fields were used to repel the drones when it came close.
This was tested using a set of 5 Firefly drones. A drone was manually designed to crash onto the rest of the drones while a function was defined to repel the other drones when it came close. Later the whole swarm was supposed to follow a preset trajectory.


[Screencast from 27-07-24 01:52:42 PM IST.webm](https://github.com/user-attachments/assets/a04d12b4-987a-46cb-a103-496e117852e5)
