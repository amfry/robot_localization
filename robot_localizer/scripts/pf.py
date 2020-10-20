#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from copy import deepcopy
from visualization_msgs.msg import Marker, MarkerArray

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time
import random

import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors
from occupancy_field import OccupancyField
from helper_functions import TFHelper

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))

    # TODO: define additional helper functions if needed

class ParticleFilter:
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
            base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            laser_subscriber: listens for new scan data on topic self.scan_topic
            tf_listener: listener for coordinate transforms
            tf_broadcaster: broadcaster for coordinate transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
    """
    def __init__(self):
        self.initialized = False        # make sure we don't perform updates before everything is setup
        rospy.init_node('pf')           # tell roscore that we are creating a new node named "pf"

        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from

        self.d_thresh = 0.25             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        self.laser_max_distance = 2.0   # maximum penalty to assess in the likelihood field model

        self.odom_pose = None
        self.scan_ranges = []
        self.robo_closest_obj = None
        self.weights = []
        self.normalized_weights = []
        self.scan_angles = []
        self.scan_distances = []
        self.scan_coordinates_map = []
        self.dist_between_pts_and_map = []
        self.bell_values = []
        self.weights = []
        self.normalized_weights = []

        self.num_particles = 5
        self.sample_num = 4

        self.resample_threshold = 1 / self.num_particles


        # TODO: define additional constants if needed

        # Setup pubs and subs

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        #
        # # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        #
        # # laser_subscriber listens for data from the lidar
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received)
        #
        # # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        ### add publisher for markers
        self.vis_pub = rospy.Publisher('visualization_markerarray', MarkerArray, queue_size=10)  # creating a publisher

        self.particle_cloud = []
        self.particle_dist = []
        #
        # # change use_projected_stable_scan to True to use point clouds instead of laser scans
        self.use_projected_stable_scan = False
        self.last_projected_stable_scan = None
        if self.use_projected_stable_scan:
            # subscriber to the odom point cloud
            rospy.Subscriber("projected_stable_scan", PointCloud, self.projected_scan_received)

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.initialized = True

        # self.marker_array = MarkerArray()

    def update_robot_pose(self, timestamp):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        # self.normalize_particles()

        # TODO: assign the latest pose into self.robot_pose as a geometry_msgs.Pose object
        # just to get started we will fix the robot's pose to always be at the origin
        self.robot_pose = Pose()

        self.transform_helper.fix_map_to_odom_transform(self.robot_pose, timestamp)

    def projected_scan_received(self, msg):
        self.last_projected_stable_scan = msg

    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.

            msg: this is not really needed to implement this, but is here just in case.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta

            #print("delta: " + str(delta))
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return
        alpha = math.atan(delta[1]/delta[0])  # angle of d[0] & d[1]

        for p in self.particle_cloud:
            beta = p.theta + alpha  # direction of delta vector in map frame
            if 0 < beta < math.pi/2:
                # Q1
                print("Q1")
                p.x = p.x + delta[0]
                p.y = p.y + delta[1]
                p.theta = p.theta + delta[2]
            if math.pi/2 <= beta < math.pi:
                # Q2
                print("Q2")
                p.x = p.x - delta[0]
                p.y = p.y + delta[1]
                p.theta = p.theta + delta[2]
            if math.pi <= beta < (3*math.pi) / 2:
                # Q3
                print("Q3")
                p.x = p.x - delta[0]
                p.y = p.y - delta[1]
                p.theta = p.theta + delta[2]
            if (3 * math.pi) / 4 <= beta < 2 * math.pi:
                # Q4
                print("Q4")
                p.x = p.x + delta[0]
                p.y = p.y - delta[1]
                p.theta = p.theta + delta[2]

        # TODO: modify particles using delta

    def map_calc_range(self,x,y,theta):
        """ Difficulty Level 3: implement a ray tracing likelihood model... Let me know if you are interested """
        # TODO: nothing unless you want to try this alternate likelihood model
        pass

    # def resample_particles(self):
    #     """ Resample the particles according to the new particle weights.
    #         The weights stored with each particle should define the probability that a particular
    #         particle is selected in the resampling step.  You may want to make use of the given helper
    #         function draw_random_sample.
    #     """
    #     # make sure the distribution is normalized
    #     self.normalize_particles()
    #     # TODO: fill out the rest of the implementation

    def update_particles_with_laser(self, msg):
        """ Updates the particle weights in response to the scan contained in the msg """
        for i in self.particle_dist:
            if not np.isnan(i):
                w = i/self.robo_closest_obj
                self.weights.append(w)
        # print("weights" + str(self.weights))

    @staticmethod
    def draw_random_sample(choices, probabilities, n):
        """ Return a random sample of n elements from the set choices with the specified probabilities
            choices: the values to sample from represented as a list
            probabilities: the probability of selecting each element in choices represented as a list
            n: the number of samples
        """
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]
        samples = []
        for i in inds:
            samples.append(deepcopy(choices[int(i)]))
        return samples

    def select_robo_scan_points(self,num):
        # num is the number of points from the scan
        # generate the angles we want to sample from scan and get those dist
        #scan points are coming from neato frame
        spacing = 360/num
        for i in range(0,num):
            angle_val = i * spacing
            self.scan_angles.append(angle_val)


    def select_robo_scan_distances(self):
        for i in self.scan_angles:
            dist_val = self.scan_ranges[int(i)]
            self.scan_distances.append(dist_val)

    def send_scan_from_base_link_to_map_frame(self, particle_obj):
    # send laser_scan points from neato frame (base link) to the map frame from 1 particle
        # print("size: " + str(len(self.scan_distances)))
        for i in range(0,len(self.scan_distances)):
            # print(i)
            r = self.scan_distances[i]
            phi = self.scan_angles[i]
            theta = particle_obj.theta
            particle_x = particle_obj.x
            particle_y = particle_obj.y
            x = r * np.cos(np.deg2rad(phi)+theta) + particle_x
            y = r * np.sin(np.deg2rad(phi)+theta) + particle_y
            val = [x, y]
            self.scan_coordinates_map.append(val)

    def scan_loc_from_particles(self):
        # project all scan points from each particle in the particle cloud
        for p in self.particle_cloud:
            # print("howdy")
            self.send_scan_from_base_link_to_map_frame(p)

    def get_scan_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5
        return marker

    def draw_scan_marker_array(self):
        print("ELECTRIC BOOGALOOOOOO")
        marker_array = MarkerArray()
        for i, s in enumerate(self.scan_coordinates_map):
            # print("i = " + str(i))
            m = self.get_scan_marker(s[0], s[1])
            print(s[0], s[1])
            m.id = i
            marker_array.markers.append(m)
        # print("w")
        self.vis_pub.publish(marker_array)
        return

    def get_dist_between_scan_point_and_map(self):
        # computes the distance between scan points projected from every particle and the nearest object on the map
        for s in self.scan_coordinates_map:
            x = s[0]
            y = s[1]
            val = self.occupancy_field.get_closest_obstacle_distance(x, y)
            self.dist_between_pts_and_map.append(val)

    def distance_to_bell_vals(self):
        self.filter_distances()
        for d in self.dist_between_pts_and_map:
            # print("distance: " + str(d))
            val = (math.e **(-1*d**(2)))**3
            self.bell_values.append(val)
        # print("---------------------")
        # # print(self.bell_values)
        # print("LENGTH: " + str(len(self.bell_values)))
        # print("---------------------")

    def bell_vals_to_weights(self):
        for i in range(0, len(self.bell_values), self.sample_num):
            vals = self.bell_values[i:i + self.sample_num-1]
            # print("vals: " + str(vals))
            self.weights.append(sum(vals) / self.sample_num)

    def filter_distances(self):
        for i in range(0,len(self.dist_between_pts_and_map)):
            if math.isnan(self.dist_between_pts_and_map[i]):
                self.dist_between_pts_and_map[i] = 10

    def normalize_weights(self):
        s = sum(self.weights)
        for w in self.weights:
            val = w / s
            self.normalized_weights.append(val)
        self.assign_weights_to_particles()

    def assign_weights_to_particles(self):
        for i in range(len(self.particle_cloud)):
            self.particle_cloud[i].w = self.normalized_weights[i]

    def resample_particles(self):
        self.particle_cloud = self.draw_random_sample(self.particle_cloud, self.normalized_weights, self.num_particles)

#### normalize those weights


    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        self.particle_cloud = []
        for i in range(0,self.num_particles):
            p = Particle(x=random.random()*5-2.5, y=random.random()*5-2.5, theta=np.random.choice(6))
            self.particle_cloud.append(p)
        self.update_robot_pose(timestamp)
        fake_particle = Particle(x=4, y=0, theta=2.904)
        self.particle_cloud[0] = fake_particle
        self.update_robot_pose(timestamp)

    def publish_particles(self):
        # msg used to be here
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map_frame),
                                  poses=particles_conv))

    def scan_received(self, msg):
        """ This is the default logic for what to do when processing scan data.
            Feel free to modify this, however, we hope it will provide a good
            guide.  The input msg is an object of type sensor_msgs/LaserScan """
        if not(self.initialized):
            # print("Case 1")
            # wait for initialization to complete
            return

        if not(self.tf_listener.canTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)):
            # print("Case 2")
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            return

        if not(self.tf_listener.canTransform(self.base_frame, self.odom_frame, msg.header.stamp)):
            # print("Case 3")
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            return

        self.scan_ranges = msg.ranges

        # calculate pose of laser relative to the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # store the the odometry pose in a more convenient format (x,y,theta)
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        if not(self.particle_cloud):
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp, self.num_particles)
        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            if self.last_projected_stable_scan:
                last_projected_scan_timeshift = deepcopy(self.last_projected_stable_scan)
                last_projected_scan_timeshift.header.stamp = msg.header.stamp
                self.scan_in_base_link = self.tf_listener.transformPointCloud("base_link", last_projected_scan_timeshift)

            #### resample section
            n.select_robo_scan_distances()
            n.scan_loc_from_particles()
            n.get_dist_between_scan_point_and_map()
            n.distance_to_bell_vals()
            n.bell_vals_to_weights()
            n.normalize_weights()
            n.resample_particles()
            n.reset_for_new_particles()
          # update based on laser scan
            self.update_robot_pose(msg.header.stamp)                # update robot's pose
            # self.resample_particles()               # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles()


    ##### visualize each particle w/ marker
    def get_marker(self, x, y, w):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.scale.x = w
        marker.scale.y = w
        marker.scale.z = w
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker

    def draw_marker_array(self):
        marker_array = MarkerArray()
        # print("he")
        for i, p in enumerate(self.particle_cloud):
            # print("i = " + str(i))
            m = self.get_marker(p.x, p.y, p.w*self.num_particles/5)
            m.id = i
            # self.vis_pub.publish(m)
            marker_array.markers.append(m)
        # print("w")
        self.vis_pub.publish(marker_array)
        # print(marker_array)
        return

    def reset_for_new_particles(self):
        self.scan_coordinates_map = []
        self.scan_distances = []
        self.dist_between_pts_and_map = []
        self.bell_values = []
        self.weights = []
        self.normalized_weights = []

if __name__ == '__main__':
    counter = 0
    n = ParticleFilter()
    r = rospy.Rate(5)
    while not(rospy.is_shutdown()):
        # in the main loop all we do is continuously broadcast the latest map to odom transform
        # print(n.initialized)
        n.transform_helper.send_last_map_to_odom_transform()
        while n.initialized:
            counter += 1
            print("counter: " + str(counter))
            time.sleep(3)
            n.initialize_particle_cloud(rospy.Time.now(), 10)
            if counter == 1:
                # only generate the sample angles 1 time
                n.select_robo_scan_points(n.sample_num)
            n.draw_marker_array()
            # n.draw_scan_marker_array()
            # print("------")
            # print(n.normalized_weights)
            # print(len(n.normalized_weights))
            # print("SUM: " + str(sum(n.normalized_weights)))
            # print("------")
            #list to clear after each set of particles in generate
            n.reset_for_new_particles()

        r.sleep()
