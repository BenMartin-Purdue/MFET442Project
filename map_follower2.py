import tf
import math
import time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from MFET44200_Lab06_Utils import Util
from simple_pid import PID
import numpy as np

class MapFollower():
    def __init__(self):
        self.waypoints = Util.parseWaypoints('knoy_demo_track.yaml')
        self.index = 1
        self.desired_waypoint = self.waypoints[self.index]
        self.previous_waypoint = self.waypoints[self.index - 1]

        rospy.init_node('Rally_car_control')
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_current_pose)
        rospy.Subscriber("/imu", Imu, self.update_imu)
        self.accel_pub = rospy.Publisher("/accelerator_cmd", Float32, queue_size=10)
        self.steer_pub = rospy.Publisher("/steering_cmd", Float32, queue_size=10)
        self.accel_pub.publish(Float32(0))

        self.current_pose = [99999,99999,0]
        self.last_imu_update_time = time.monotonic()
        self.last_amcl_angle = 0

        # PID controller
        # CONTROL PARAMETERS #
        accel_Kp = 1
        accel_Ki = .001
        accel_Kd = .009

        steer_Kp = 1
        steer_Ki = .001
        steer_Kd = .009

        self.target_vel = 0
        # CONTROL PARAMETERS # 

        self.accel_PID = PID(accel_Kp, accel_Ki, accel_Kd, 0)               # import the parameters into the PID controller
        self.steering_PID = PID(steer_Kp, steer_Ki, steer_Kd, 0)            # Set point is 0 here, because later calculations evaluate to 0 when on base.

    def update_current_pose(self, msg) -> None:
        #############################################################################################
        # Updates the current position by taking data from the amcl localization, and extracts usable
        # data from the output data object.
        #############################################################################################    
        quaternion = (
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,    # extracting data from the pose
			msg.pose.pose.orientation.z,    # to a tuple for processing
			msg.pose.pose.orientation.w
        )
        
        euler = tf.transformations.euler_from_quaternion(quaternion)    # I dont know what this does
        yaw = math.degrees(euler[2])        
        # rospy.loginfo("AMCL RAW : {0}".format(yaw))   
        self.last_amcl_angle = yaw
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        # rospy.loginfo("Self.ImuOffset : {0}".format(self.imu_offset))
        self.last_imu_update_time = time.monotonic()
        

    def update_imu(self, msg) -> None:
        update_time = time.monotonic() - self.last_imu_update_time
        self.last_imu_update_time = time.monotonic()

        quaternion = (
			msg.orientation.x,
			msg.orientation.y,    # extracting data from the pose
			msg.orientation.z,    # to a tuple for processing
			msg.orientation.w
        )
        
        euler = tf.transformations.euler_from_quaternion(quaternion)    # I dont know what this does
        yaw = math.degrees(euler[2])
        # rospy.loginfo("IMU RAW : {0}".format(yaw))
        #imu_rot = yaw - self.last_amcl_angle
        # rospy.loginfo("Imu_rot : {0}".format(imu_rot))

    def control_loop(self, waypoint_tol : float = 2, will_loop=True):

        dist_to_waypoint = Util.calculate_linear_dist(self.current_pose, self.desired_waypoint)
        theta_simple = math.degrees(np.arcsin((self.desired_waypoint[1] - self.current_pose[1]) / dist_to_waypoint))

        if (self.current_pose[0] < self.desired_waypoint[0]) & (self.current_pose[1] < self.desired_waypoint[1]):
            theta_global = theta_simple
        elif (self.current_pose[0] >= self.desired_waypoint[0]) & (self.current_pose[1] < self.desired_waypoint[1]):
            theta_global = 180 - theta_simple
        elif (self.current_pose[0] < self.desired_waypoint[0]) & (self.current_pose[1] >= self.desired_waypoint[1]):
            theta_global = theta_simple
        elif (self.current_pose[0] >= self.desired_waypoint[0]) & (self.current_pose[1] >= self.desired_waypoint[1]):
            theta_global = -180 - theta_simple

        theta_final = Util.addAngles(theta_global, -self.current_pose[2])
    
        
        theta_PID = self.steering_PID(theta_final)

        theta_steering = -Util.clamp(theta_PID, max=50, min=-50)
        theta_steering = Float32(theta_steering * 41)
        self.steer_pub.publish(theta_steering)

        # DEBUG STEERING INFO #
        rospy.loginfo("Current Heading / Global car angle: {0}".format(self.current_pose[2]))
        rospy.loginfo("Theta_simple / Calculated crcsin : {0}".format(theta_simple))
        rospy.loginfo("Theta_global / Global angle to Waypoint : {0}".format(theta_global))
        rospy.loginfo("Theta_final / Car Angle to Waypoint : {0}".format(theta_final))
        rospy.loginfo("Theta_PID / Value out of the PID : {0}".format(theta_PID))
        rospy.loginfo("Theta_steering / Value sent to the steering : {0}\n".format(theta_steering))
        # DEBUG STEERING INFO # 

         # GATEWAY CONDITION #
        if(dist_to_waypoint > waypoint_tol):
            # should the distance to the desired waypoint be greater than the tolerance, exit now.
            return
        
        # LOOPBACK CONDITIONS #
        if((self.index + 1 >= len(self.waypoints)) and will_loop == True): 
            # if index is larger than the waypoint array, reset the index back to 0
            # basically, will call a continous looping of the waypoints if true
            self.index = -1 # <- will have one added to it - making it 0
        elif (self.index + 1 >= len(self.waypoints) and will_loop == False):
            # HACK: This avoids an indexOutOfBoundsException, but never actually closes down the 
            #   control loop.
            # TODO: Actually close the control loop.
            # stops the loop from updating if the end of the array has been reached            
            return

        # Stepping the desired waypoint 
        self.index += 1                                             # step the index
        self.previous_waypoint = self.desired_waypoint              # save the previous desired waypoint
        self.desired_waypoint = self.waypoints[self.index] # set the new desired waypoint

def main():
    map_follower_instance = MapFollower()
    while not rospy.is_shutdown():
        map_follower_instance.control_loop(waypoint_tol=.5)

if __name__ == "__main__":
    main()        
