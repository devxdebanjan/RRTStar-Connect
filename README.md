## 🎯AIM
Implementing a path planning algorithm on an image of 600x600 pixels. I have used 
the <b><i>RRT* connect</b></i> algorithm for path planning in my code. Next the path is traversed using 
PID by turtlesim node in _**ROS noetic**_. 
## 👨‍💻LANGUAGE
Python
## 💻OS 
Linux (made in Ubuntu)
## 📚LIBRARIES USED
- OpenCV     
- math 
- random 
- matplotlib 
- rospy 
- math
## ⚙️INSTRUCTIONS
- Implementing a path planning algorithm on an image of 600x600 pixels. I have used the RRT* connect algorithm for path planning in my code. Next the path is traversed using PID by turtlesim node in ROS noetic. 
- Here there are three files: 
<code>spawnturt.py</code>, <code>goto.py</code> and <code>myconnect.py</code>. <code>spawnturt.py</code> spawns the turtle at the start 
position of the path. <code>goto.py</code> traverse is the whole path coordinates and moves the turtle 
forward through PID. <code>myconnect.py</code> is the actual script which contents the whole 
algorithmic implementation of RRT Star connect. 
<br><br>
Do these steps before running <code>myconnect.py</code>: <br>
- For running the ROS noetic, open your terminal and run roscore.  
- Next you can choose to kill the turtle1 because the spawned turtle will be entirely different.<br> 
- Command to do this: <br><code>rosservice call /kill 'turtle1'</code>

Now run <code>myconnect.py</code>. <br><br>Few additional tips: <br>
- In lines: 13, 42 and 311, exchange the directories of the image you would want to generate a 
path in. 
- You can easily alter the parameters within the first five line of the code. 
    - Increasing 
the step size decreases the time required for getting the path. 
    - Increasing the search radius 
increases the rewiring radius of the path nodes. 
    - Increasing safety leads you to a safer path 
where it is ensured that no obstacles are there in the way. <br>
    - Ideal safety is half of the step 
parameter but that takes too much Max_iterations. 
- So suitable step size and safety is given 
for a smooth path generating of Image1.png.

This runs the whole algorithm and then gives you a path from start to end point. As soon as 
you close the OpenCV window the turtle should run and the same implication can be seen in 
matplotlib window. if turtle misbehaves;) try to **change parameters like tolerance, or coefficient of linear/angular velocity**. Suitable parameters are given for a smooth path 
generation of Image1.png for now.

The output paths generated and turtle running on them can be seen in the output folder of the repo.

Please leave if a follow if you found this useful.<br>
Thank you✨