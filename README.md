# Implementing_A_star_on_Turtlebot3

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