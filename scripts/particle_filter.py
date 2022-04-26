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


# The size of the world
world_size = 385

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(list, n):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # we want to give the array a corresponding probability value to the element in list
    prob = []
    # keeps count of the item in the probability array
    count = 0
    # iterates through list to get the probability
    for i in list:
        prob[count] = i.w
        count += 1
    # gives a random sample given the list and its probability for each item and 
    # how many items to return
    return np.random.choice(list, n ,prob)


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

        self.map = data
    

    def initialize_particle_cloud(self):

        for i in range(self.num_particles):
            # Gets a random position for the particle
            # scales the integer by size of the world
            pose = Point(np.random.randint(-60, 5) * world_size, np.random.randint(-50, 10) * world_size, 0)
            # gets a random quaternion value array
            quant = quaternion_from_euler(0, 0, random()*2*math.pi)
            # for every particle in the particle cloud, we want to
            # initialize the particle to a random position
            particle = Particle(Pose(), 1)
            particle.pose.position = pose
            particle.pose.orientation = Quaternion(quant[0], quant[1], quant[2], quant[3])
            # appends the new particle to the cloud
            self.particle_cloud.append(particle)

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # keeps track of the total weight of all the particles
        total_weight = 0
        # calculating the total weight by adding the weights of
        # all the particles in the particle cloud
        for i in self.particle_cloud:
            total_weight += i.w
         
        # normalizing the total weight to make sure that all the weights add up to 1
        for j in self.particle_cloud:
            j.w = j.w/total_weight


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
        # resamples cloude using the draw_random_sample function
        self.particle_cloud = draw_random_sample(self.particle_cloud, self.num_particles)
        # need to normalize particles after resampling
        self.normalize_particles()


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

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        # compute average of all particles locations & orientation.
        self.normalize_particles()

        # find the sum of all the x y coordinates of the particles
        total_x, total_y = 0, 0

        # iterates through the particles in the particle cloud to find the sum of x and
        # y positions of the particles
        for i in self.particle_cloud:
            # updates the position of particles
            total_x += i.pose.position.x
            total_y += i.pose.position.y
        
        # the yaw using the average total x and y values
        new_yaw = math.atan2(total_y/self.num_particles, total_x/self.num_particles)
        
        # finding the average of x and y sums and changing the robot_estimated position
        self.robot_estimate.position = Pose(total_x/self.num_particles, total_y/self.num_particles, new_yaw)
        self.robot_estimate.orientation = new_yaw

        # updating this estimated position
        self.publish_estimated_robot_pose()


    
    def update_particle_weights_with_measurement_model(self, data):
        # need to add sound and orientation

        # resamples the existing particles first
        self.resample_particles()

        # keeps track of the diff of the position and orientation of 
        # laser data and particle positions
        diff_x, diff_y, orientation = 0, 0, 0

        # goes through each particle and finds the difference
        for i in self.particle_cloud:
            diff_x = np.abs(data[i].pose.position.x - i.pose.position.x)
            diff_y = np.abs(data[i].pose.position.y - i.pose.position.y)
            orientation = np.abs(data[i].pose.orientation.z - i.pose.orientation.z)
            # updates the weights using Monte Carlo Localization Algorithm
            i.w = 1/(diff_x + diff_y + orientation)
        


    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        self.resample_particles()

        prev_x = self.odom_pose_last_motion_update.pose.position.x
        prev_y = self.odom_pose_last_motion_update.pose.position.y
        prev_o = self.odom_pose_last_motion_update.pose.position.z

        curr_x = self.odom_pose.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        curr_o = self.odom_pose.pose.position.z

        ## ideal cases in odom
        delt_rot_1 = math.atan2(curr_y - prev_y, curr_x - prev_x) - prev_o
        delt_trans = math.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2)
        delt_rot_2 = curr_o - prev_o - delt_rot_1

        # incorporating noise
        # alpha value
        a1 = 0.05
        a2 = 15.0*3.14 / 180.0
        a3 = 0.05
        a4 = 0.01

        sample1 = a1*delt_rot_1 + a2*delt_trans
        sample2 = a3 * delt_trans + a4*(delt_rot_1 + delt_rot_2)
        sample3 = a1 * delt_rot_2 + a2*delt_trans

        delt_rot_1_noise = delt_rot_1 - np.random.normal(0, sample1**2)
        delt_trans_noise = delt_trans - np.random.normal(0, sample2**2)
        delt_rot_2_noise = delt_rot_2 - np.random.normal(0, sample3**2)

        for i in self.particle_cloud:
            # new_position
            i.pose.position.x = i.pose.position.x + delt_trans_noise * math.cos(prev_o + delt_rot_1_noise)
            i.pose.position.y = i.pose.position.y + delt_trans_noise * math.sin(prev_o + delt_rot_1_noise)
            i.pose.position.z = i.pose.position.z + delt_rot_1_noise + delt_rot_2_noise



if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









