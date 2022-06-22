#!/usr/bin/env python3
# license removed for brevity
from pickletools import uint8
import rospy
import math
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty

def clear_costmap():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:

        clear = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
        clear()
        print('Costmap cleared')
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    return

class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        self.current_waypoint = rospy.get_param('waypoint_num') 
        points_seq_temp = rospy.get_param('move_base_seq/p_seq')
        points_seq =list()
        for i in  range(3*self.current_waypoint,len(points_seq_temp)):
            points_seq.append(float(points_seq_temp[i])) 
       

      
               

        # Only yaw angle required (no ratotions around x and y axes) in deg:
        yaweulerangles_seq_temp = rospy.get_param('move_base_seq/yea_seq')
        yaweulerangles_seq =list()
        for i in  range(self.current_waypoint,len(yaweulerangles_seq_temp)):
            yaweulerangles_seq.append(float(yaweulerangles_seq_temp[i]))
        
        
        #List of goal quaternions:
        quat_seq = list()

        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = self.current_waypoint

        #n =int(rospy.get_param('move_base_seq/no_waypoints'))
        n=3
        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
            
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, float(yawangle)*math.pi/180, axes='sxyz'))))

        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        
        rospy.loginfo(str(points))
        rospy.loginfo(str(yaweulerangles_seq))
        rospy.loginfo(str(quat_seq))
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
        rospy.set_param('/move_base_en',True)

        clear_costmap()
        self.movebase_client()

 

  
            
    def active_cb(self):
        #stop =rospy.get_param('/stop_robot')
        rospy.loginfo("Goal pose " + str(self.goal_cnt+1)+" is now being processed by the Action Server...")
      

    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))

        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        rospy.set_param('/move_base_en', True)
        rospy.set_param('waypoint_num',self.goal_cnt)
        stop =rospy.get_param('/stop_robot')
        if stop:
            rospy.set_param('/move_base_en', False)
            self.client.cancel_all_goals()
            rospy.loinfo("OBSTACLE IN THE WAY")
            os._exit(0)
           
            
            
    def done_cb(self, status, result):
        self.goal_cnt += 1
        
       
        
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.set_param('/move_base_en', False)
            stop =rospy.get_param('/stop_robot')
            #if stop:
            #    rospy.set_param('/move_base_en', False)
            #    self.client.cancel_all_goals()
            #    clear_costmap()
            #    rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")
            #   os._exit(0)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")
            os._exit(0)
        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt-self.current_waypoint]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.set_param('/goal_reached',True)
                rospy.signal_shutdown("Final goal pose reached!")
                
                return

        if status == 4 :
            rospy.set_param('/move_base_en', False)
            rospy.loginfo("Aborted")
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            
            return

        if status == 5:
            
            rospy.set_param('/move_base_en', False)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8 :
            rospy.set_param('/move_base_en', False)
            #stop =rospy.get_param('/stop_robot')
            #if stop:
            #    rospy.set_param('/move_base_en', False)
            #    self.client.cancel_all_goals()
            #    clear_costmap()
            #    rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")
            #    os._exit(0)
                

    def movebase_client(self):
      
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        rospy.set_param('/move_base_en', True)
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    print("Starting off")
    robo_stop =rospy.get_param('/stop_robot')
    goal_reached =rospy.get_param('/goal_reached')
    if(not goal_reached and not robo_stop):
        print("Set 2D Nav Pose in RViz.")
        rospy.set_param('/move_base_en',True)  
        rospy.sleep(1)
        

        try:  
            MoveBaseSeq()
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation finished.")
    else : print("Obstacle in the way or goal reached")
    os._exit(0)