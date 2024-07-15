import rclpy
from rclpy.node import Node 
import json
import os
import numpy as np

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from mmr_base.msg import SpeedProfilePoint, SpeedProfilePoints
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point


class PublishMap(Node):

    cones = Marker()
    waypoints = Marker()
    speed_profile_points = SpeedProfilePoints()

    def __init__(self):
        super().__init__('publish_map')
        self.load_parameters()

        self.cones_positions_publisher = self.create_publisher(Marker,self.cones_topic,1)
        self.trajectory_points_publisher = self.create_publisher(Marker,self.trajectory_points_topic,1)
        self.speed_profile_publisher = self.create_publisher(SpeedProfilePoints, self.speed_profile_topic, 10)

        self.get_logger().info("Publish map node initialized")
        self.init_cones_markers()
        self.init_waypoints_markers()

        self.export_cones()
        self.export_waypoints()
        self.export_speed_profile_points()

        t = self.timer_period     #20Hz
        self.timer = self.create_timer(t,self.timer_callback)


    def init_cones_markers(self):
        #same as in ekf_odometry
        self.cones.header.frame_id = "track"
        self.cones.ns = "WaypointsAbsolutePos"
        self.cones.id = 0
        self.cones.type = Marker.SPHERE_LIST
        self.cones.action = Marker.ADD
        self.cones.scale.x = 0.35
        self.cones.scale.y = 0.35
        self.cones.scale.z = 0.35
        self.cones.pose.position.x = 0.0
        self.cones.pose.position.y = 0.0
        self.cones.pose.position.z = 0.0
        #Initialize with identity quaternion (no rotation)
        self.cones.pose.orientation.w = 1.0
        self.cones.pose.orientation.x = 0.0
        self.cones.pose.orientation.y = 0.0
        self.cones.pose.orientation.z = 0.0

    def init_waypoints_markers(self):
        
        self.waypoints.header.frame_id = "track"
        self.waypoints.ns = "ConesAbsolutePos"
        self.waypoints.id = 0
        self.waypoints.type = Marker.SPHERE_LIST
        self.waypoints.action = Marker.ADD
        self.waypoints.scale.x = 0.35
        self.waypoints.scale.y = 0.35
        self.waypoints.scale.z = 0.35
        self.waypoints.pose.position.x = 0.0
        self.waypoints.pose.position.y = 0.0
        self.waypoints.pose.position.z = 0.0
        #Initialize with identity quaternion (no rotation)
        self.waypoints.pose.orientation.w = 1.0
        self.waypoints.pose.orientation.x = 0.0
        self.waypoints.pose.orientation.y = 0.0
        self.waypoints.pose.orientation.z = 0.0

    def load_parameters(self):
        self.declare_parameter('timer_period')
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('cones_topic')
        self.cones_topic = self.get_parameter('cones_topic').value
        self.declare_parameter('trajectory_points_topic')
        self.trajectory_points_topic = self.get_parameter('trajectory_points_topic').value
        self.declare_parameter('speed_profile_topic')
        self.speed_profile_topic = self.get_parameter('speed_profile_topic').value

        self.declare_parameter('cones_file')
        self.cones_file = os.environ['HOME'] + self.get_parameter('cones_file').value
        self.declare_parameter('trajectory_points_file')
        self.trajectory_points_file = os.environ['HOME'] + self.get_parameter('trajectory_points_file').value
        self.declare_parameter('speed_profile_file')
        self.speed_profile_file = os.environ['HOME'] + self.get_parameter('speed_profile_file').value

    def export_cones(self):
        with open(self.cones_file) as f_cones_pos:
            cones_positions = json.load(f_cones_pos)
            
            blue_x = cones_positions["blue_x"]
            blue_y = cones_positions["blue_y"]

            yellow_x = cones_positions["yellow_x"]
            yellow_y = cones_positions["yellow_y"]

            orange_x = cones_positions["orange_x"]
            orange_y = cones_positions["orange_y"]

            big_orange_x = cones_positions["big_orange_x"]
            big_orange_y = cones_positions["big_orange_y"] 

        
        for x,y in zip(blue_x,blue_y):
            p = Point()
            p.x = x
            p.y = y 
            p.z = 0.0

            c = ColorRGBA()
            c.r = 0.0
            c.g = 0.0
            c.b = 1.0
            c.a = 1.0

            self.cones.points.append(p)
            self.cones.colors.append(c)     

        for x,y in zip(yellow_x,yellow_y):
            p = Point()
            p.x = x
            p.y = y 
            p.z = 0.0

            c = ColorRGBA()
            c.r = 1.0
            c.g = 1.0
            c.b = 0.0
            c.a = 1.0

            self.cones.points.append(p)
            self.cones.colors.append(c)  
                   
        if len(orange_x) != 0:              #in some bags orange cones are not present
            for x,y in zip(orange_x,orange_y):
                p = Point()
                p.x = x
                p.y = y 
                p.z = 0.0

                c = ColorRGBA()
                c.r = 1.0
                c.g = 0.3
                c.b = 0.0
                c.a = 1.0

                self.cones.points.append(p)
                self.cones.colors.append(c)  
            

        if len(big_orange_x) != 0:
            for x,y in zip(big_orange_x,big_orange_y):
                p = Point()
                p.x = x
                p.y = y 
                p.z = 0.0

                c = ColorRGBA()
                c.r = 1.0
                c.g = 0.63
                c.b = 0.1
                c.a = 1.0

                self.cones.points.append(p)
                self.cones.colors.append(c)  
    

    def export_waypoints(self):
        with open(self.trajectory_points_file, 'r') as f_waypoints:
            waypoints = json.load(f_waypoints)
            waypoints_x = waypoints["X"]
            waypoints_y = waypoints["Y"]

        for x,y in zip(waypoints_x,waypoints_y):
            p = Point()
            p.x = x
            p.y = y 
            p.z = 0.0

            #white
            c = ColorRGBA()
            c.r = 1.0
            c.g = 1.0
            c.b = 1.0
            c.a = 1.0

            self.waypoints.points.append(p)
            self.waypoints.colors.append(c) 

    def export_speed_profile_points(self):
        arr = np.genfromtxt(self.speed_profile_file,
                 delimiter=";",skip_header = 1)
        
        points_list = [] #list of SpeedProfilePoint 
        for a in arr:
            p = SpeedProfilePoint()
            p.point = Point(x=a[1], y = a[2]) # le coordinate x e y del punto sono nelle posizioni 1 e 2 dell'array
            p.ackerman_point = AckermannDrive(speed = a[5],acceleration = a[6], steering_angle = a[3])
            points_list.append(p)

        self.speed_profile_points.points = points_list
        
            
        

    def timer_callback(self):
        self.cones_positions_publisher.publish(self.cones)
        self.trajectory_points_publisher.publish(self.waypoints)
        self.speed_profile_publisher.publish(self.speed_profile_points)

        self.get_logger().info("Published cones,waypoints and speedprofilepoints")


def main(args=None):
    rclpy.init(args=args)
    node = PublishMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
