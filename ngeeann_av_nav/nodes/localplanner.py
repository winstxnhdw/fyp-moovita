#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose2D
from ngeeann_av_nav.msg import Path2D, State2D
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Float64
from utils.heading2quaternion import heading_to_quaternion
from utils.cubic_spline_interpolator import generate_cubic_path

class LocalPathPlanner:

    def __init__(self):

        ''' Class constructor to initialise the class '''

        # Initialise publishers
        self.local_planner_pub = rospy.Publisher('/ngeeann_av/path', Path2D, queue_size=10)
        self.path_viz_pub = rospy.Publisher('/nggeeann_av/viz_path', Path, queue_size=10)
        self.target_vel_pub = rospy.Publisher('/ngeeann_av/target_velocity', Float64, queue_size=10)

        # Initialise subscribers
        self.goals_sub = rospy.Subscriber('/ngeeann_av/goals', Path2D, self.goals_cb, queue_size=10)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=10)
        self.gridmap_sub = rospy.Subscriber('/map', OccupancyGrid, self.gridmap_cb, queue_size=10)

        # Load parameters
        try:
            self.planner_params = rospy.get_param("/local_path_planner")
            self.frequency = self.planner_params["update_frequency"]
            self.frame_id = self.planner_params["frame_id"]
            self.target_vel_def = self.planner_params["target_velocity"]
            self.car_width = self.planner_params["car_width"]

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.halfpi = np.pi / 2
        self.ds = 0.1
        self.origin_x = 0
        self.origin_y = 0

        # Class variables to use whenever within the class when necessary
        self.target_vel = self.target_vel_def
        self.ax = []
        self.ay = []
        self.gmap = OccupancyGrid()

    def goals_cb(self, msg):

        ''' Callback function to receive waypoint data from the Global Path Planner '''

        self.ax = []
        self.ay = []
        
        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            self.ax.append(px)
            self.ay.append(py)

        print("\nGoals received: {}".format(len(msg.poses)))

    def vehicle_state_cb(self, msg):

        ''' Callback function to receive information on the vehicle's vertical and horizontal coordinates '''

        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta

    def gridmap_cb(self, msg):

        self.gmap = msg

    def determine_path(self, cx, cy, cyaw):

        width = self.gmap.info.width
        height = self.gmap.info.height
        resolution = self.gmap.info.resolution
        origin_x = self.origin_x
        origin_y = self.origin_y
        collisions = []
        collide_id = None

        # Checks points along path
        for n in range(150, len(cyaw) - 150):

            # Draws swath of the vehicle
            for i in np.arange(-0.5 * self.car_width, 0.5 * self.car_width, resolution):

                ix = int((cx[n] + i*np.cos(cyaw[n] - 0.5 * np.pi) - origin_x) / resolution)
                iy = int((cy[n] + i*np.sin(cyaw[n] - 0.5 * np.pi) - origin_y) / resolution)
                p = iy * width + ix

                if (self.gmap.data[p] != 0):
                    collisions.append(n)
                    #print('\nPotential collision at ({}, {})'.format(cx[n], cy[n]))
        
        if len(collisions) != 0:
            cx, cy, cyaw = self.collision_avoidance(collisions, cx, cy, cyaw)

        return cx, cy, cyaw

    def collision_reroute(self, cx, cy, cyaw, collisions, opening_dist):
        
        collide_id = collisions[0]
        collide_id_end = collisions[-1]

        # Points to leave path
        dev_x1= cx[collide_id - 150]
        dev_y1 = cy[collide_id - 150]

        dev_x2 = cx[collide_id - 75]
        dev_y2 = cy[collide_id - 75]

        # Point to intersect path
        intersect_x1 = cx[collide_id_end + 75]
        intersect_y1 = cy[collide_id_end + 75]

        intersect_x2 = cx[collide_id_end + 150]
        intersect_y2 = cy[collide_id_end + 150]

        # Point of avoidance from collision
        avoid_x1 = cx[collide_id] + opening_dist*np.cos(cyaw[collide_id] - 0.5 * np.pi)
        avoid_y1 = cy[collide_id] + opening_dist*np.sin(cyaw[collide_id] - 0.5 * np.pi)
        
        avoid_x2 = cx[collide_id_end] + opening_dist*np.cos(cyaw[collide_id_end] - 0.5 * np.pi)
        avoid_y2 = cy[collide_id_end] + opening_dist*np.sin(cyaw[collide_id_end] - 0.5 * np.pi)

        reroute_x = [dev_x1, dev_x2, avoid_x1, avoid_x2, intersect_x1, intersect_x2]
        reroute_y = [dev_y1, dev_y2, avoid_y1, avoid_y2, intersect_y1, intersect_y2]
        
        rcx, rcy, rcyaw = generate_cubic_path(reroute_x, reroute_y, self.ds)

        # stiching to form new path
        cx   = np.concatenate(( cx[0 : collide_id - 151], rcx, cx[(collide_id_end + 151) : ] ))
        cy   = np.concatenate(( cy[0 : collide_id - 151], rcy, cy[(collide_id_end + 151) : ] ))
        cyaw = np.concatenate(( cyaw[0 : collide_id - 151], rcyaw, cyaw[(collide_id_end + 151) : ] ))

        print('Generated dev path')
        return cx, cy, cyaw

    def collision_avoidance(self, collisions, cx, cy, cyaw):

        opening_width = 0
        opening_id = 0
        opening_dist = 0
        collide_view = []

        resolution = self.gmap.info.resolution
        width = self.gmap.info.width
        height = self.gmap.info.height
        origin_x = self.origin_x
        origin_y = self.origin_y

        collide_id = collisions[0]
            
        for i in np.arange(-10, 10, 0.1):
            ix = int((cx[collide_id] + i*np.cos(cyaw[collide_id] - 0.5 * np.pi) - origin_x) / resolution)
            iy = int((cy[collide_id] + i*np.sin(cyaw[collide_id] - 0.5 * np.pi) - origin_y) / resolution)
            p = iy * width + ix
            collide_view.append(self.gmap.data[p])
        
        print('\nCollision window constructed.')
        opening_width, opening_id = self.find_opening(collide_view)
        opening_width = opening_width * 0.1
        opening_dist = (opening_id - 100) * 0.1

        print('Predicting possible avoidance...')

        if opening_width < self.car_width:
            print('Avoidance is not possible.\nInitiating emergency brakes.\n')
            self.target_vel = 0
        
        else:
            print('Avoidance is possible.\nCommencing avoidance manoeuvre.\n')
            self.target_vel = self.target_vel_def

        print('Obstacle Length: {}'.format(0.1*len(collisions)))
        print('Detected opening of width: {}'.format(opening_width))
        print('Detected distance to opening: {}'.format(opening_dist))

        cx, cy, cyaw = self.collision_reroute(cx, cy, cyaw, collisions, opening_dist)
        return cx, cy, cyaw

    def find_opening(self, arr):
        ''' 
        Recieves an array representing the view perpendicular to the path at the point of predicted collision
        Returns index of midpoint of largest opening, and size
        '''

        count = 0 
        result = 0
        idx = 0

        for i in range(0, len(arr)): 
            if (arr[i] >= 10): #threshold value
                count = 0

            else: 
                count+= 1

                if (count > result):
                    idx = i

                result = max(result, count)

        idx = int(round(idx - result / 2.0))   #midpoint of largest opening
        return result, idx

    def create_pub_path(self):

        ''' Uses the cubic_spline_planner library to interpolate a cubic spline path over the given waypoints '''

        cx, cy, cyaw, _ = generate_cubic_path(self.ax, self.ay, self.ds)
        cx, cy, cyaw = self.determine_path(cx, cy, cyaw)

        cells = min(len(cx), len(cy), len(cyaw))

        target_path = Path2D()
        
        viz_path = Path()
        viz_path.header.frame_id = "map"
        viz_path.header.stamp = rospy.Time.now()

        for n in range(0, cells):
            # Appending to Target Path
            npose = Pose2D()
            npose.x = cx[n]
            npose.y = cy[n]
            npose.theta = cyaw[n]
            target_path.poses.append(npose)

            # Appending to Visualization Path
            vpose = PoseStamped()
            vpose.header.frame_id = self.frame_id
            vpose.header.seq = n
            vpose.header.stamp = rospy.Time.now()
            vpose.pose.position.x = cx[n]
            vpose.pose.position.y = cy[n]
            vpose.pose.position.z = 0.0
            vpose.pose.orientation = heading_to_quaternion(np.pi * 0.5 - cyaw[n])
            viz_path.poses.append(vpose)

        self.local_planner_pub.publish(target_path)
        self.path_viz_pub.publish(viz_path)

def main():

    ''' Main function to initialise the class and node. '''

    # Initialise the class
    local_planner = LocalPathPlanner()

    # Initialise the node
    rospy.init_node('local_planner')

    # Set update rate
    r = rospy.Rate(local_planner.frequency) 

    # Wait for messages
    rospy.wait_for_message('/ngeeann_av/goals', Path2D)
    rospy.wait_for_message('/map', OccupancyGrid)

    while not rospy.is_shutdown():
        try:
            local_planner.create_pub_path()
            local_planner.target_vel_pub.publish(local_planner.target_vel)

            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()