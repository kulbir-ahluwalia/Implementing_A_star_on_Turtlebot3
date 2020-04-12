# Implementing_A_star_on_Turtlebot3 - Phase 3

Enter all values in metres and degrees.
 
## How to Run
1. Clone this repo or extract the zip file "proj3_group13_gazebo_python". <br>

2. To run the code, navigate to the "proj3_group13_gazebo_python/Phase_3" folder. From the terminal, run the command `python3 Turtlebot_A_star.py 
` <br>
3. Enter all values that the program asks for in metres and degrees.If the points provided are in the obstacle space, program will terminate.<br>
4. To view the simulation of "video1_orange.avi" enter the following parameters -   
-Start_x, start_y, start_angle (entered separately) : (-4, -3.5, 30)  
-Goal_x, Goal_y : (0.5, -2.5)      
-rpm1, rpm2 = 40,50   
-radius of rigid robot = 0.105   
-Clearance : 0.2   
Open the video "output.avi"    

5. To view the simulation of "video2_black.avi" enter the following parameters -   
-Start_x, start_y, start_angle (entered separately) : (-4, -4, 30)    
-Goal_x, Goal_y :  (4, 2.5)     
-rpm1, rpm2 = 40,50    
-radius of rigid robot = 0.105   
-Clearance : 0.2    
Open the video "output.avi"   

6. Close the figure generated after the program ends.




## Organisation of code
The code is split into 2 python programs which are:-
1. **Turtlebot_A_star.py** is main python file which has the A star algorithm, code for non-holonomic constraints, video generation and plotting.
 
2. **obstacle_check.py** checks whether a point is within an obstacle or not.
 
## Output
The simulation video is output as **output.avi** and the frames are available in the "plots" folder.
The final image is "plots.png" in the plots folder.

## System and library requirements.
 - Python3
 - Numpy
 - OpenCV
 - Math
 - Matplotlib
 - Time
  
 ## Installation 
 Above mentioned dependencies can be installed using the following links:   
 Numpy -https://docs.scipy.org/doc/numpy-1.10.1/user/install.html   
 OpenCV -https://docs.opencv.org/master/d5/de5/tutorial_py_setup_in_windows.html   
 Math - https://datatofish.com/install-package-python-using-pip/   
 Matplotlib - https://matplotlib.org/users/installing.html   
 

# Implementing_A_star_on_Turtlebot3 - Phase 4
 
 
1. First of all, this project is done in ROS melodic (ubuntu 18.04).
2. Have a catkin workspace ready with build, devel, src and .catkin_workspace.
3. Clone/copy the repository: https://github.com/kulbir-ahluwalia/Implementing_A_star_on_Turtlebot3
4. Clone it at or paste it at the src folder of catkin-ws: username@pcname:~/catkin_ws/src$
5. Change directory to catkin_ws by running the following command in new terminal: cd catkin_ws
6. Then run another command in the same terminal: catkin_make
7. The next command is (if you are using bash) : source devel/setup.bash
Important Note: Have turtle bot 3 installed in your workspace already before you proceed to next step. If you install it now then start over from the step 5.
8. We use the following command to run the Gazebo world: roslaunch a_star_turtlebot a_star_launch_turtlebot.launch x_pos:=-2 y_pos:=1 Yaw:=0.0
Here enter the start coordinates (x-coordinate goes at x_pos, y-coordinate goes at y_pos, Yaw takes angles in radians)
9. This should spawn the turtlebot3 launched in the Gazebo at the desired start coordinates.
10. Open another terminal.
11. Change directory to catkin_ws by running the following command: cd catkin_ws
12. The next command is (if you are using bash) : source devel/setup.bash
13. Now run the following command to execute the python script: rosrun a_star_turtlebot dummy.py
Now this make the robot move to the goal node.