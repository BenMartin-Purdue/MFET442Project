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
        self.waypoints = Util.parseWaypoints('knoy_demo_track.yaml')    # Read in waypoints from file
        self.index = 1                                                  # select the SECOND waypoint as the target (Car is placed at the first)
        self.desired_waypoint = self.waypoints[self.index]              # set the SECOND waypoint as the desired target
        self.previous_waypoint = self.waypoints[self.index - 1]         # LEGACY: set the first waypoint as the last waypoint visited

        rospy.init_node('Rally_car_control')                                                # init the control node
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_current_pose) # read in data from AMCL
        rospy.Subscriber("/imu", Imu, self.update_imu)                                      # read in data from the IMU
        self.accel_pub = rospy.Publisher("/accelerator_cmd", Float32, queue_size=10)        # setup a publisher for the acceleration controls
        self.steer_pub = rospy.Publisher("/steering_cmd", Float32, queue_size=10)           # setub a publisher for the steering controls
        self.accel_pub.publish(Float32(0))  

        self.current_pose = [99999,99999,0]             # before pose is update, set it to a value that would never be reached
        self.last_imu_update_time = time.monotonic()    # store the startup time for the IMU before it is updated

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
        
        euler = tf.transformations.euler_from_quaternion(quaternion)    # I dont know what this does exactly,
        yaw = math.degrees(euler[2])                                    # but it somehow turns the quat into usable angles

        # THE INTERPOLATION UPDATE # 
        self.IMU_drift = Util.addAngles(self.current_pose[2], -yaw)     # takes the current pose (updated by the IMU) and figures out how far off the IMU is
        # THE INTERPOLATION UPDATE # 


        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]   # Store the important parts of the AMCL data for use
        self.last_imu_update_time = time.monotonic()                                    # update the IMU's update timer
        

    def update_imu(self, msg) -> None:
        update_time = time.monotonic() - self.last_imu_update_time      # figure out how long its been since the last update
        # self.last_imu_update_time = time.monotonic()                    # reset the IMU update timer

        quaternion = (
			msg.orientation.x,
			msg.orientation.y,    # extracting data from the pose
			msg.orientation.z,    # to a tuple for processing
			msg.orientation.w
        )
        
        euler = tf.transformations.euler_from_quaternion(quaternion)    # I dont know what this does
        yaw = math.degrees(euler[2])                                    # but it somehow turns the quat into usable angles
        yaw = Util.addAngles(yaw, -self.IMU_drift)                      # takes the value calculated by AMCL for IMU error and subtracts it from the IMU value (to zero it out)
        self.current_pose[2] = yaw


    def control_loop(self, waypoint_tol : float = 2, will_loop=True):
        dist_to_waypoint = Util.calculate_linear_dist(self.current_pose, self.desired_waypoint)                         # find out how far away the waypoint is
        theta_simple = math.degrees(np.arcsin((self.desired_waypoint[1] - self.current_pose[1]) / dist_to_waypoint))    # calculate the angle to the waypoint

        if (self.current_pose[0] < self.desired_waypoint[0]) & (self.current_pose[1] < self.desired_waypoint[1]):       # depending on the quad the angle resides in, 
            theta_global = theta_simple                                                                                 # either don't do anything, its already global
        elif (self.current_pose[0] >= self.desired_waypoint[0]) & (self.current_pose[1] < self.desired_waypoint[1]):
            theta_global = 180 - theta_simple                                                                           # or transform the angle to global space
        elif (self.current_pose[0] < self.desired_waypoint[0]) & (self.current_pose[1] >= self.desired_waypoint[1]):
            theta_global = theta_simple                                                                                 # no transform
        elif (self.current_pose[0] >= self.desired_waypoint[0]) & (self.current_pose[1] >= self.desired_waypoint[1]):
            theta_global = -180 - theta_simple                                                                          # transform

        theta_final = Util.addAngles(theta_global, -self.current_pose[2])   # to convert global angle to car angle, subtract the car's current angle
    
        theta_PID = self.steering_PID(theta_final)                          # send this data to the PID controller

        theta_steering = -Util.clamp(theta_PID, max=50, min=-50)            # invert and clamp the data
        theta_steering = Float32(theta_steering * 41)                       # convert to usable data for the steering controller
        self.steer_pub.publish(theta_steering)                              # send the data to the steering controller

        # DEBUG STEERING INFO #
        """
        rospy.loginfo("Current Heading / Global car angle: {0}".format(self.current_pose[2]))
        rospy.loginfo("Theta_simple / Calculated crcsin : {0}".format(theta_simple))
        rospy.loginfo("Theta_global / Global angle to Waypoint : {0}".format(theta_global))
        rospy.loginfo("Theta_final / Car Angle to Waypoint : {0}".format(theta_final))
        rospy.loginfo("Theta_PID / Value out of the PID : {0}".format(theta_PID))
        rospy.loginfo("Theta_steering / Value sent to the steering : {0}\n".format(theta_steering))
        """
        # DEBUG STEERING INFO # 

        # DEBUG INTERP INFO # 
        rospy.loginfo("Current interpolated angle : {0}".format(self.current_pose[2]))
        rospy.loginfo("Current expected IMU drift : {0}".format(self.IMU_drift))
        rospy.loginfo("Time since last AMCL update : {0}".format(time.monotonic() - self.last_imu_update_time))

        # DEBUG INTERP INFO

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
            #   control loop. So instead, I just throw an interupt error.
            # Technically, this works, but isn't ideal
            self.steer_pub.publish(Float32(0))
            self.steer_accel.publish(Float32(0))   
            raise InterruptedError

        # Stepping the desired waypoint 
        self.index += 1                                             # step the index
        self.previous_waypoint = self.desired_waypoint              # save the previous desired waypoint
        self.desired_waypoint = self.waypoints[self.index]          # set the new desired waypoint

def main():
    map_follower_instance = MapFollower()
    while not rospy.is_shutdown():
        map_follower_instance.control_loop(waypoint_tol=.5)

if __name__ == "__main__":
    main()        
