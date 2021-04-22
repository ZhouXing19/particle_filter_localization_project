#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random

# Import likelihood_field.py
from likelihood_field import LikelihoodField
import sys



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(arr):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # TODO
    # Uses random_sample function from numpy.random
    #sample = self.num_particles * random_sample() + 1

    # Uses random.choice to populate a random sample of num_particles elements from particle_cloud based on p
    arr = np.random.choice(arr, size=len(arr), p=[part.w for part in arr])
    return


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()
        self.map_origin = None
        self.map_height, self.map_width = None, None
        self.map_resolution_factor = None 

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []
        

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None
        self.odom_pose = None # Will update this in self.robot_scan_received()


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):
        '''
        data: OccupancyGrid

        The map will be built in the OccupancyGrid message. Instead of a two dimensional grid, the data is stored in a one-dimensional vector (OccupancyGrid.data). We translate a pair of integer coordinates with the two_d_2_one_d_idx(x,y,size_x) function in geometry.py

        The size_x, in this case, should be the width of the map: https://answers.ros.org/question/205521/robot-coordinates-in-map/
        '''


        # print(f"in get_map, type(data) = {type(data)}") #  <class 'nav_msgs.msg._OccupancyGrid.OccupancyGrid'>

        # https://www.programcreek.com/python/example/95997/nav_msgs.msg.OccupancyGrid (example 3)

        self.map = data
        self.map_origin = np.array([data.info.origin.position.x, data.info.origin.position.y])
        self.map_width, self.map_height = data.info.width, data.info.height
        self.map_resolution_factor = 1 / data.info.resolution

        # print(f"map_origin: {self.map_origin}, map_size: ({self.map_width},{self.map_height}), \
        #         map_resolution_factor: {self.map_resolution_factor}") 
                # map_origin: [-10. -10.], map_size: (384,384), map_resolution_factor: 19.99999970197678


        # print(f"type of self.map.data: {type(self.map.data)}") # tuple
        # print(f"size of self.map.data: {len(self.map.data)}") # 147456

    def two_d_2_one_d_idx(self, x, y, width):
        # The data are saved as 1-D tuples. To convert the 2-D tuple to 1-D: 
        # https://github.com/RobotTeaching/COMP4034/wiki/Workshop-6:-Minitask-4-(Marked)#task-4-keep-track-of-your-moves-via-mapping
        return x + y * width

    def abs_loc_2_map_loc(self, x, y):
        pose_in_map_x = int((x - self.map_origin[0]) * self.map_resolution_factor)
        pose_in_map_y = int((y - self.map_origin[1]) * self.map_resolution_factor)
        return pose_in_map_x, pose_in_map_y

    

    def initialize_particle_cloud(self):
        # TODO

        
        # Need to create an instance of the LikelihoodField class before calling its methods
        lf = LikelihoodField()

        # Get the upper and lower x and y range
        ((x_lb, x_ub), (y_lb, y_ub)) = lf.get_obstacle_bounding_box()

        # For each particle, set a random position and orientation (uniformly distributed)
        for i in range(self.num_particles):
            # https://www.programcreek.com/python/example/88501/geometry_msgs.msg.Pose
            new_pose = Pose()

            # Positions
            new_pose.position = Point()
            new_pose.position.x = np.random.uniform(x_lb, x_ub)
            new_pose.position.y = np.random.uniform(y_lb, y_ub)

            # Evaluate if the new_pose is out of map

            # https://answers.ros.org/question/201172/get-xy-coordinates-of-an-obstacle-from-a-map/
            # https://answers.ros.org/question/10268/where-am-i-in-the-map/
            # ocuGrid.data, -1 means this position is unknown: https://answers.ros.org/question/207914/occupancy-grid-coordinates/
            


            pose_in_map_x, pose_in_map_y = self.abs_loc_2_map_loc(new_pose.position.x, new_pose.position.y)
            cur_idx = self.two_d_2_one_d_idx(pose_in_map_x, pose_in_map_y, self.map_width)

            while self.map.data[cur_idx] == -1:
                new_pose.position.x = np.random.uniform(x_lb, x_ub)
                new_pose.position.y = np.random.uniform(y_lb, y_ub)
                pose_in_map_x, pose_in_map_y = self.abs_loc_2_map_loc(new_pose.position.x, new_pose.position.y)
                cur_idx = self.two_d_2_one_d_idx(pose_in_map_x, pose_in_map_y, self.map_width)


            new_pose.position.z = 0

            # Orientations
            new_pose.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, np.random.uniform(0, 361))
            new_pose.orientation.x = q[0]
            new_pose.orientation.y = q[1]
            new_pose.orientation.z = q[2]
            new_pose.orientation.w = q[3]

            # Add the particle to the cloud with initial weight 1
            new_particle = Particle(new_pose, 1.0)
            self.particle_cloud.append(new_particle)



        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        # TODO
        # Get the current sum of the particle weights
        particle_weight_total = sum([part.w for part in self.particle_cloud])

        # For each particle, normalize its weight by dividing it by the sum total of all weights
        for part in self.particle_cloud:
            part.w = part.w / particle_weight_total

        return


    def publish_particle_cloud(self):
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        # TODO

        # Use draw_random_sample() here

        return


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
            frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
            frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model(curr_x - old_x, curr_y - old_y, curr_yaw - old_yaw)

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # TODO
        return

    def update_particle_weights_with_measurement_model(self, data):

        # TODO
        return


    def update_particles_with_motion_model(self, dx, dy, dyaw):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO
        # 1. Get the delta in x, y and yaw (we can also get this from the parameters)
        '''cur_x, cur_y = self.odom_pose.pose.position.x, self.odom_pose.pose.position.y
        prev_x, prev_y = self.odom_pose_last_motion_update.pose.position.x, self.odom_pose_last_motion_update.pose.position.y
        cur_yaw = get_yaw_from_pose(self.odom_pose.pose)
        prev_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        dx, dy, dyaw = cur_x - prev_x, cur_y - prev_y, cur_yaw - prev_yaw'''

        # 2. For all particles, apply the delta 
        for curr_particle in self.particle_cloud:

            # Get the original info of the particle
            this_particle = curr_particle
            this_pose = this_particle.pose
            this_yaw = get_yaw_from_pose(this_pose)

            # Apply simple addition to get the updated position / orientation
            new_x = this_pose.position.x + dx
            new_y = this_pose.position.y + dy
            new_yaw = this_yaw + dyaw
            q = quaternion_from_euler(0, 0, new_yaw)

            # Create a new pose with the updated configurations
            new_pose = Pose()
            new_pose.orientation = Quaternion()
            new_pose.position.x = new_x
            new_pose.position.y = new_y
            new_pose.orientation.x = q[0]
            new_pose.orientation.y = q[1]
            new_pose.orientation.z = q[2]
            new_pose.orientation.w = q[3]
            
            # Create a new particle based on the new pose, and replace the
            # Old particle with the new one.
            new_particle = Particle(new_pose, this_particle.w)
            curr_particle = new_particle

        return

            



if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









