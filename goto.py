#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import spawnturt
import matplotlib.pyplot as plt
#spawning turtle at initial point of path
def spawning(path):
    startx=path[0][0]/60
    starty=path[0][1]/60
    spawnturt.spawn_turtle(startx,starty,0,'gototurt')

def turtshow(str,x,y):
    
    plt.plot(x,y,'bo')
    # Display the loaded image
    plt.title('Loaded PNG Image')
    

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/gototurt/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/gototurt/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/gototurt/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/gototurt/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=10):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self,path,imgname):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        image = plt.imread(imgname)
        plt.imshow(image)

        # inserting a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = float(0.1)
        vel_msg = Twist()
        for i in range(len(path)-1):
            point=path[i+1]
            goal_pose.x = float(point[0]/60)
            goal_pose.y = float(point[1]/60)
            while self.euclidean_distance(goal_pose) >= distance_tolerance:

                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(goal_pose)

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)
                print(self.pose)
                xt=self.pose.x*60
                yt=self.pose.y*60
                plt.plot(xt,yt,'co')
                plt.pause(0.01)
                # Publish at the desired rate.
                self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__=='__main__':   #running a test case
    path=[(545, 535), (534, 532), (516, 511), (503, 496), (492, 473), (492, 433), (494, 423), (491, 384), (503, 359), (511, 333), (514, 320), (518, 292), (512, 253), (484, 230), (477, 226), (440, 214), (431, 176), (395, 160), (361, 140), (325, 158), (300, 164), (272, 172), (245, 174), (225, 169), (207, 163), (186, 157), (173, 156), (142, 132), (108, 112), (84, 81), (55, 55)]
    spawning(path)
    x = TurtleBot()
    x.move2goal(path,'Image2.png')