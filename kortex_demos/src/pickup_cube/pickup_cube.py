#!/usr/bin/env python
import argparse
import sys
import time
import threading

from collections import deque

import cv2
import roslib
import rospy
import math
import copy

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Float64, Int32, Bool

from kortex_demos.srv import *
from kortex_driver.msg import BaseCyclic_Feedback, ActionType, ActionNotification, ConstrainedPose, ActionEvent, Finger, GripperMode
from kortex_driver.srv import Base_ClearFaults, ExecuteAction, ExecuteActionRequest, SetCartesianReferenceFrame, OnNotificationActionTopic, OnNotificationActionTopicRequest, SendGripperCommand, SendGripperCommandRequest, GetMeasuredCartesianPose, SendTwistCommand, SendTwistCommandRequest, Stop

class PickupDemo:
    
    # Initialization functions

    def __init__(self):

        self.robot_name = "my_gen3"
        
        self.BASE_FRAME = 1
        self.TOOL_FRAME = 2
        self.OPENED_GRIPPER_POSITION = 0.01
        self.CLOSED_GRIPPER_POSITION = 0.60

        # TS = Translation_Speed
        self.TRANSLATION_SPEED = 0.15

        # OS = Orientation_Speed
        self.ORIENTATION_SPEED = 20

        self.BIG_RADIUS = 0.25
        self.MEDIUM_RADIUS = 0.10
        self.SMALL_RADIUS = 0.01
        self.NULL_RADIUS = 0.001
        self.SMALL_RADIUS_ANGLE = 1

        self.ANGULAR_SPEED_LIMIT = 10

        # Cartesian poses we need
        self.pose_starting = Twist(Vector3(0.352,-0.032,0.410), Vector3(179.900,0.001,90.000))
        self.pose_transition = Twist(Vector3(0.40,0.011,0.166), Vector3(179.900,0.001,90.000))

        # Gripper positions we need
        self.gripper_opened = 0.0
        self.gripper_closed = 0.60

        # Statistics
        self.start_time = time.time()
        self.elapsed_time = self.start_time

        # Services
        self.init_services()

        # Subscribers

        self.actual_tool_pose  = Twist()
        self.basefeedback = rospy.Subscriber("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback, self.base_feedback_callback, buff_size=1)

        self.is_cube_found = False
        self.sub_cube_found = rospy.Subscriber('is_cube_found', Bool, self.is_cube_found_callback, queue_size=1)

        # Events
        self.action_type = ActionType.UNSPECIFIED_ACTION
        self.is_action_successful = False
        self.is_action_completed = threading.Event()
        self.is_action_completed.clear()

        # Subscribe to the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.action_notification_callback)

    def init_services(self):
        # Services
        clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
        rospy.wait_for_service(set_cartesian_reference_frame_full_name)
        self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

        activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
        rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
        self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

        send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        get_measured_cartesian_pose_full_name = '/' + self.robot_name + '/base/get_measured_cartesian_pose'
        rospy.wait_for_service(get_measured_cartesian_pose_full_name)
        self.get_measured_cartesian_pose = rospy.ServiceProxy(get_measured_cartesian_pose_full_name, GetMeasuredCartesianPose)

        stop_action_full_name = '/' + self.robot_name + '/base/stop'
        rospy.wait_for_service(stop_action_full_name)
        self.stop_the_robot = rospy.ServiceProxy(stop_action_full_name, Stop)

        self.twist_command = SendTwistCommandRequest()
        send_twist_command_full_name = '/' + self.robot_name + '/base/send_twist_command'
        rospy.wait_for_service(send_twist_command_full_name)
        self.send_twist_command = rospy.ServiceProxy(send_twist_command_full_name, SendTwistCommand)

        rospy.wait_for_service("get_xyz_coordinates")
        self.srv_get_xyz_coordinates = rospy.ServiceProxy('get_xyz_coordinates', GetCoordinates)

        rospy.wait_for_service("get_cube_width")
        self.srv_get_cube_width = rospy.ServiceProxy('get_cube_width', GetCubeWidth)

    # Callbacks
    
    def action_notification_callback(self, notif):
        if self.action_type == notif.handle.action_type:
            if notif.action_event == ActionEvent.ACTION_END:
                self.is_action_completed.set()
                self.is_action_successful = True
            elif notif.action_event == ActionEvent.ACTION_ABORT:
                self.is_action_completed.set()
                self.is_action_successful = False

    def base_feedback_callback(self, feedback):
        self.actual_tool_pose.linear.x = feedback.base.tool_pose_x
        self.actual_tool_pose.linear.y = feedback.base.tool_pose_y
        self.actual_tool_pose.linear.z = feedback.base.tool_pose_z
        self.actual_tool_pose.angular.x = feedback.base.tool_pose_theta_x
        self.actual_tool_pose.angular.y = feedback.base.tool_pose_theta_y
        self.actual_tool_pose.angular.z = feedback.base.tool_pose_theta_z

    def is_cube_found_callback(self, msg):
        self.is_cube_found = msg.data
    
    # Functions to calculate stuff

    def get_current_base_frame_pose(self):

        # Get the pose from the arm
        try:
            response = self.get_measured_cartesian_pose()
        except rospy.ServiceException:
            rospy.logerr("Failed to get measured cartesian pose")

        current_base_frame_pose = Twist()

        # This condition is to work around a bug where the linear and angular values are sometimes mixed up
        # Won't be needed in the near future
        if abs(response.output.x) < 1.5 and abs(response.output.y) < 1.5 and abs(response.output.z) < 3\
        and (abs(response.output.theta_x) > 1 or abs(response.output.theta_y) > 1 or abs(response.output.theta_z) > 1):

            current_base_frame_pose.linear.x = response.output.x
            current_base_frame_pose.linear.y = response.output.y
            current_base_frame_pose.linear.z = response.output.z

            current_base_frame_pose.angular.x = response.output.theta_x
            current_base_frame_pose.angular.y = response.output.theta_y
            current_base_frame_pose.angular.z = response.output.theta_z

        else :
            current_base_frame_pose.linear.x = response.output.theta_x
            current_base_frame_pose.linear.y = response.output.theta_y
            current_base_frame_pose.linear.z = response.output.theta_z

            current_base_frame_pose.angular.x = response.output.x
            current_base_frame_pose.angular.y = response.output.y
            current_base_frame_pose.angular.z = response.output.z
        
        return current_base_frame_pose

    def calculate_camera_x_y(self, z_cloud):
        # Linear regression found experimentally
        x = 0.00004 * (z_cloud * z_cloud) + 0.00001 * z_cloud + 0.317
        y = 0.00018 * (z_cloud * z_cloud * z_cloud) - 0.00333 * (z_cloud * z_cloud) + 0.009 * z_cloud + 0.385

        return (x, y)

    def get_camera_target_point(self):
        coordinates = self.srv_get_xyz_coordinates().point
        x_pixels = coordinates.x
        y_pixels = coordinates.y
        z_cloud = coordinates.z #m

        # Convert the pixels
        # This could be much much better
        x = x_pixels * 0.001
        y = y_pixels * 0.001

        return (x, y, z_cloud)

    def calculate_delta_pose(self, target_pose, reference_frame):

        if reference_frame == self.TOOL_FRAME:
            
            (x_target, y_target, z_cloud) = self.get_camera_target_point()
            (x_camera, y_camera) = self.calculate_camera_x_y(z_cloud)

            delta_x = x_camera - x_target
            delta_y = y_camera - y_target
            delta_z = (z_cloud - 0.195) # we want to end up at 19.5cm from the cube

            delta_xtheta = 0
            delta_ytheta = 0
            delta_ztheta = 0

        elif reference_frame == self.BASE_FRAME:
            
            current_base_frame_pose = self.get_current_base_frame_pose()

            delta_x = target_pose.linear.x - current_base_frame_pose.linear.x
            delta_y = target_pose.linear.y - current_base_frame_pose.linear.y
            delta_z = target_pose.linear.z - current_base_frame_pose.linear.z

            delta_xtheta = target_pose.angular.x - current_base_frame_pose.angular.x
            delta_ytheta = target_pose.angular.y - current_base_frame_pose.angular.y
            delta_ztheta = target_pose.angular.z - current_base_frame_pose.angular.z

        else: # TODO we should deal with an erroneous frame here 
            pass

        return (delta_x, delta_y, delta_z, delta_xtheta, delta_ytheta, delta_ztheta)

    def normalize_vector3(self, x, y, z):

        temp = (x * x) + (y * y) + (z * z)

        if temp == 0:
            temp = 0.001

        norm = math.sqrt(temp)

        x_normalized = x / norm
        y_normalized = y / norm
        z_normalized = z / norm

        return (x_normalized, y_normalized, z_normalized)

    def is_position_reached(self,target_pose,radius,reference_frame):

        # Get feedback depending on frame
        if reference_frame == self.TOOL_FRAME:
            feedback = self.actual_tool_pose
            (target_pose.linear.x, target_pose.linear.y, feedback.linear.z) = self.get_camera_target_point()
            target_pose.linear.z = 0.195
            (feedback.linear.x, feedback.linear.y) = self.calculate_camera_x_y(feedback.linear.z)
        elif reference_frame == self.BASE_FRAME:
            feedback = self.get_current_base_frame_pose()
        else: #TODO handle error if frame is not base or tool 
            pass
        
        # Check if we reached the position with a certain offset
        radius_theta = radius * 100
        if  (target_pose.linear.x - radius) < feedback.linear.x and feedback.linear.x < (target_pose.linear.x + radius) and \
            (target_pose.linear.y - radius) < feedback.linear.y and feedback.linear.y < (target_pose.linear.y + radius) and \
            (target_pose.linear.z - radius) < feedback.linear.z and feedback.linear.z < (target_pose.linear.z + radius) and \
            (target_pose.angular.x - radius_theta) < feedback.angular.x and feedback.angular.x < (target_pose.angular.x + radius_theta) and \
            (target_pose.angular.y - radius_theta) < feedback.angular.y and feedback.angular.y < (target_pose.angular.y + radius_theta) and \
            (target_pose.angular.z - radius_theta) < feedback.angular.z and feedback.angular.z < (target_pose.angular.z + radius_theta):
            return True
        else: 
            return False

    def is_twist_command_null(self):
        return self.twist_command.input.twist.linear_x == 0 and \
                self.twist_command.input.twist.linear_y == 0 and \
                self.twist_command.input.twist.linear_z == 0 and \
                self.twist_command.input.twist.angular_x == 0 and \
                self.twist_command.input.twist.angular_y == 0 and \
                self.twist_command.input.twist.angular_z == 0
    
    # Functions to move the robot

    def reach_cartesian_pose_and_wait_for_completion(self, target_pose):

        # Clear the faults
        self.clear_faults()
        rospy.sleep(0.5)

        # Create the pose and the request
        my_constrained_pose = ConstrainedPose()
        my_constrained_pose.target_pose.x = target_pose.linear.x
        my_constrained_pose.target_pose.y = target_pose.linear.y
        my_constrained_pose.target_pose.z = target_pose.linear.z
        my_constrained_pose.target_pose.theta_x = target_pose.angular.x
        my_constrained_pose.target_pose.theta_y = target_pose.angular.y
        my_constrained_pose.target_pose.theta_z = target_pose.angular.z

        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)

        # Tell the callback to look for a REACH_POSE action type
        self.action_type = ActionType.REACH_POSE
        
        # Send the pose
        rospy.loginfo("Sending pose...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose")
        else:
            rospy.loginfo("Waiting for pose to finish...")

        # Wait for the pose to finish
        success = self.is_action_completed.wait()

        # Clear the notifications variables afterwards
        self.is_action_completed.clear()
        self.action_type = ActionType.UNSPECIFIED_ACTION

        # Return if action was successful or aborted
        return self.is_action_successful

    def send_gripper_command_and_wait_for_completion(self, value):
        
        # Create the gripper command request
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            rospy.loginfo("Sending the gripper position...")

        # Wait a bit
        rospy.sleep(0.5)
        return True
  
    def process_and_send_twist_to_robot(self, twist, use_normalized_twist, translation_speed_limit, orientation_speed_limit, reference_frame):

        if use_normalized_twist:
            (direction_x, direction_y, direction_z) = self.normalize_vector3(twist.linear.x, twist.linear.y, twist.linear.z)
            (direction_x_theta, direction_y_theta, direction_z_theta) = self.normalize_vector3(twist.angular.x, twist.angular.y, twist.angular.z)
            target = Twist(Vector3(direction_x, direction_y, direction_z), Vector3(direction_x, direction_y_theta, direction_z_theta))
        else:
            target = twist
            p_gain = 1
            minimum_translation_speed = 0.01
            minimum_orientation_speed = 1
            #rospy.loginfo(target)

        if use_normalized_twist:
            pass
        else:
            # Validation: put to 0 if lower than minimum, cap at max if above max, multiply by a proportional gain
            target.linear.x *= p_gain
            target.linear.x = target.linear.x if abs(target.linear.x) > minimum_translation_speed else 0.0
            target.linear.x = math.copysign(min(abs(target.linear.x), translation_speed_limit), target.linear.x)
            target.linear.y *= p_gain            
            target.linear.y = target.linear.y if abs(target.linear.y) > minimum_translation_speed else 0.0
            target.linear.y = math.copysign(min(abs(target.linear.y), translation_speed_limit), target.linear.y)
            target.linear.z *= p_gain            
            target.linear.z = target.linear.z if abs(target.linear.z) > minimum_translation_speed else 0.0
            target.linear.z = math.copysign(min(abs(target.linear.z), translation_speed_limit), target.linear.z)
            target.angular.z *= p_gain
            target.angular.z = target.angular.z if abs(target.angular.z) > minimum_orientation_speed else 0.0
            target.angular.z = math.copysign(min(abs(target.angular.z), orientation_speed_limit), target.angular.z)
            target.angular.z *= -1 #hardcoded -1 here

        # Fill the twist command for the arm
        self.twist_command.input.twist.linear_x = target.linear.x
        self.twist_command.input.twist.linear_y = target.linear.y
        self.twist_command.input.twist.linear_z = target.linear.z
        self.twist_command.input.twist.angular_x = 0.0
        self.twist_command.input.twist.angular_y = 0.0
        self.twist_command.input.twist.angular_z = target.angular.z

        # Send the command to the arm        
        self.twist_command.input.reference_frame = reference_frame
        self.twist_command.input.duration = 0
        self.send_twist_command(self.twist_command)
        
    def move_toward_pose(self, target_pose, translation_speed, orientation_speed, radius, reference_frame):

        # Find the delta between current pose and target pose
        (delta_x, delta_y, delta_z, delta_xtheta, delta_ytheta, delta_ztheta) = self.calculate_delta_pose(target_pose, reference_frame)
        delta_pose = Twist(Vector3(delta_x, delta_y, delta_z), Vector3(delta_xtheta, delta_ytheta, delta_ztheta))

        (direction_x, direction_y, direction_z) = self.normalize_vector3(delta_pose.linear.x, delta_pose.linear.y, delta_pose.linear.z)
        (direction_x_theta, direction_y_theta, direction_z_theta) = self.normalize_vector3(delta_pose.angular.x, delta_pose.angular.y, delta_pose.angular.z)
        target = Twist(Vector3(direction_x, direction_y, direction_z), Vector3(direction_x, direction_y_theta, direction_z_theta))

        if self.is_position_reached(target_pose, radius, reference_frame):
            if self.is_position_reached(target_pose, self.MEDIUM_RADIUS, reference_frame):
                if self.is_position_reached(target_pose, 0.05, reference_frame):
                    self.process_and_send_twist_to_robot(delta_pose, False, 0.08, 30, reference_frame)
                else:
                    self.process_and_send_twist_to_robot(delta_pose, False, 0.12, 30, reference_frame)
            else:
                self.process_and_send_twist_to_robot(delta_pose, False, 0.16, 30, reference_frame)
        else:
            self.process_and_send_twist_to_robot(delta_pose, False, 0.20, 30, reference_frame)

        return self.is_twist_command_null()
    
    # Demo steps

    def rotate_camera(self,translation_speed,orientation_speed,radius):
        
        widths = []
        zthetas = []
        previous_width = 9999
        rotation = 10 #deg

        i = 0
        # Rotate one way
        while not rospy.is_shutdown() :
            # Get the width of the square
            w = self.srv_get_cube_width()
            if i == 0:
                previous_width = w.data
            
            # Change angular z target
            target_pose = copy.deepcopy(self.actual_tool_pose)
            target_pose.angular.z -= rotation

            # Move until you find the minimum width
            while not (self.actual_tool_pose.angular.z < (target_pose.angular.z + 7)) and not rospy.is_shutdown():
                self.move_toward_pose(target_pose, translation_speed, orientation_speed, radius, self.BASE_FRAME)

            # If the width grows bigger two times in a row, break from the loop
            w = self.srv_get_cube_width()
            if previous_width < w.data:
                i = i + 1
                if i > 2:
                    break
            else:
                i = 0

        # Rotate the other way
        previous_width = 9999
        while not rospy.is_shutdown() :
            # Get the width of the square
            w = self.srv_get_cube_width()
            if i == 0:
                previous_width = w.data

            # Change angular z target
            target_pose = copy.deepcopy(self.actual_tool_pose)
            target_pose.angular.z += rotation

            widths.append(w.data)
            zthetas.append(self.actual_tool_pose.angular.z)

            # Move until you find the minimum width
            while not (self.actual_tool_pose.angular.z > (target_pose.angular.z - 7)) and not rospy.is_shutdown():
                self.move_toward_pose(target_pose, translation_speed, orientation_speed, radius, self.BASE_FRAME)

            # If the width grows bigger two times in a row, break from the loop
            w = self.srv_get_cube_width()
            if previous_width < w.data:
                i = i + 1
                if i > 2:
                    break
            else:
                i = 0

        # Find where the minimum width was and go there
        minwidth = min(widths)
        index = widths.index(minwidth)
        target_ztheta = zthetas[index]

        target_pose = copy.deepcopy(self.get_current_base_frame_pose())
        target_pose.angular.z = target_ztheta
        self.reach_cartesian_pose_and_wait_for_completion(target_pose)

        return True

    def approach(self, translation_speed, orientation_speed, radius):

        # Approach the cube broadly and reach the good XY coordinates
        while not rospy.is_shutdown() :

            curr = self.get_current_base_frame_pose()
            target_pose = copy.deepcopy(curr)

            # If we see the cube
            if self.is_cube_found:
                (target_pose.linear.x, target_pose.linear.y, target_pose.linear.z) = self.get_camera_target_point()
                self.move_toward_pose(target_pose, translation_speed, orientation_speed, radius, self.TOOL_FRAME)
                # we approached the cube enough
                if 0.205 >= target_pose.linear.z and target_pose.linear.z >= 0.185 and self.is_twist_command_null():
                    break

            # If we don't see the cube
            else : 
                self.stop_the_robot()
                rospy.sleep(0.1)
                while not rospy.is_shutdown():
                    self.move_toward_pose(self.pose_starting, translation_speed, orientation_speed, radius, self.BASE_FRAME)
                    # Look for the cube
                    if self.is_cube_found:
                        # We found the cube
                        break

        # Approach the cube more precisely in Z
        target_pose = copy.deepcopy(self.get_current_base_frame_pose())

        target_pose.linear.z -= 0.07 # approach the cube by 7mm
        start = time.time()
        time_elapsed = 0 #seconds
        while not rospy.is_shutdown() :
            time_elapsed = time.time() - start
            if self.move_toward_pose(target_pose, 0.03, 0, 0.02, self.BASE_FRAME) or time_elapsed > 5:
                break

        self.stop_the_robot()

    def main(self):

        # Start the demo
        while not rospy.is_shutdown():
            
            # Initialize to starting position
            print ("Going to the initial position...")
            self.reach_cartesian_pose_and_wait_for_completion(self.pose_starting)
            self.send_gripper_command_and_wait_for_completion(self.gripper_opened)
            print ("Initial position reached.")

            # Rotate camera to find grasping angle
            rospy.loginfo("Finding the cube...")
            self.rotate_camera(self.TRANSLATION_SPEED,self.ORIENTATION_SPEED,self.SMALL_RADIUS)
            rospy.loginfo("Found the cube.")
            time.sleep(1)

            # Approach
            rospy.loginfo("Approaching the cube...")
            self.approach(self.TRANSLATION_SPEED,self.ORIENTATION_SPEED,self.BIG_RADIUS)
            rospy.loginfo("Finished the approach.")

            # Grab the cube
            rospy.loginfo("Grabbing the cube...")
            success = self.send_gripper_command_and_wait_for_completion(self.gripper_closed)
            rospy.loginfo("Finished grabbing.")
            time.sleep(1)

            # Move and drop the cube
            rospy.loginfo("Dropping the cube...")
            self.reach_cartesian_pose_and_wait_for_completion(self.pose_transition)
            self.send_gripper_command_and_wait_for_completion(self.gripper_opened)
            rospy.loginfo("Cube dropped.")

if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('pickup_cube')
    
    # Create the demo object
    demo = PickupDemo()
    try:
        demo.main()
    except KeyboardInterrupt:
        print("Shutting down")
        demo.stop_the_robot()
        cv2.destroyAllWindows()
