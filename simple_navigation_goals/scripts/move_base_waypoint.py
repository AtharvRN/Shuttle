#!/usr/bin/env python3
# license removed for brevity
from pickletools import uint8
import rospy
import math
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus,GoalID
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
#from std_msgs.msg import Bool,UInt8


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
       
        points_seq_temp = rospy.get_param('move_base_seq/p_seq')
        points_seq =list()
        for points in  points_seq_temp:
            points_seq.append(float(points))
       

      
               

        # Only yaw angle required (no ratotions around x and y axes) in deg:
        yaweulerangles_seq_temp = rospy.get_param('move_base_seq/yea_seq')
        yaweulerangles_seq =list()
        for points in  yaweulerangles_seq_temp:
            yaweulerangles_seq.append(float(points))
        
        
        #List of goal quaternions:
        quat_seq = list()

        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0

        #n =int(rospy.get_param('move_base_seq/no_waypoints'))
        n=3
        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
            
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, float(yawangle)*math.pi/180, axes='sxyz'))))

        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        
        rospy.loginfo(str(points))
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        #print(self.pose_seq)
        rospy.loginfo(str(self.pose_seq))

        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        #wait = self.client.wait_for_server(rospy.Duration(5.0))
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        
        #self.way_pub= rospy.Publisher("waypoint_no", UInt8, queue_size=10)
        #self.move_en= rospy.Publisher("move_base_enable", Bool, queue_size=10)
        #self.stopper = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        #self.twist =Twist()
        #self.twist.linear.x =0
        #self.twist.linear.y =0
        #self.twist.linear.z =0
        #self.twist.angular.x=0
        #self.twist.angular.y=0
        #self.twist.angular.z=0
        #self.cancel =rospy.Publisher("/move_base/cancel",GoalID,queue_size=1)
        #self.gid =GoalID()
        #self.gid.id ={}
    
        self.movebase_client()

    #def callback(self,data):
    #    if(data is False):
    #        print("NO obstacle")
    #        return 
    #    elif (data is True) :
    #        print("Obstacle spotted")

            
    #        twist =Twist()
    #        twist.linear.x =0
    #        twist.linear.y =0
    #        twist.linear.z =0
    #        twist.angular.x=0
    #        twist.angular.y=0
    #        twist.angular.z=0
    #        
    #        self.stopper.publish(twist)
    #        rospy.Subscriber("stop", Bool, self.callback)
    #       
    #        self.cancel.publish({})

    #        return#self.way_pub= rospy.Publisher("waypoint_no", UInt8, queue_size=10)
        #self.move_en= rospy.Publisher("move_base_enable", Bool, queue_size=10)
        #self.stopper = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        #self.twist =Twist()
        #self.twist.linear.x =0
        #self.twist.linear.y =0
        #self.twist.linear.z =0
        #self.twist.angular.x=0
        #self.twist.angular.y=0
        #self.twist.angular.z=0
        #self.cancel =rospy.Publisher("/move_base/cancel",GoalID,queue_size=1)
        #self.gid =GoalID()
        #self.gid.id ={}
    
        self.movebase_client()

    #def callback(self,data):
    #    if(data is False):
    #        print("NO obstacle")
    #        return 
    #    elif (data is True) :
    #        print("Obstacle spotted")

            
    #        twist =Twist()
    #        twist.linear.x =0
    #        twist.linear.y =0
    #        twist.linear.z =0
    #        twist.angular.x=0
    #        twist.angular.y=0
    #        twist.angular.z=0
    #        
    #        self.stopper.publish(twist)
    #        rospy.Subscriber("stop", Bool, self.callback)
    #       
    #        self.cancel.publish({})

    #        return

 


            
    def active_cb(self):
        stop =rospy.get_param('/stop_robot')
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")
        if stop:
            self.client.cancel_all_goals()
            return


    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))

        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        rospy.set_param('/move_base_en', True)
        rospy.set_param('waypoint_num',self.goal_cnt)
        stop =rospy.get_param('/stop_robot')
        if stop:
            print("stopping")
            #self.stopper.publish(twist)
            self.client.cancel_all_goals()
            #self.cancel.publish(self.gid)
            sys.exit()
            print("stopped")
            
            
        #self.way_pub.publish(self.goal_cnt)
        #self.move_en.publish(True)


    def done_cb(self, status, result):
        self.goal_cnt += 1
        stop =rospy.get_param('/stop_robot')
        if stop:
            #self.stopper.publish(self.twist)
            self.client.cancel_all_goals()
            return
        #rospy.loginfo("stop ",stop)
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2 or stop:
            rospy.set_param('/move_base_en', False)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                rospy.set_param('/goal_reached',True)
                return

        if status == 4 or stop:
            rospy.set_param('/move_base_en', False)
            rospy.loginfo("Aborted")
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            
            return

        if status == 5 or stop:
            #self.move_en.publish(False)
            rospy.set_param('/move_base_en', False)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8 or stop:
            rospy.set_param('/move_base_en', False)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
    #for pose in pose_seq:   
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        
        #self.move_en.publish(True)
        #rospy.Subscriber("stop", Bool, self.callback)
        rospy.set_param('/move_base_en', True)
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    #print("Starting off")
    print("Set 2D Nav Pose in RViz")
    #rospy.sleep(10)
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
