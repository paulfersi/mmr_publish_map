import rclpy
from rclpy.node import Node 
import json
import os

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

config = {
    "conesPositionsTopic" : "/slam/cones_positions",
    "conesPositionsFilePath" : os.environ['HOME'] + "/mmr-drive/src/0_common/save_map/save_map/data/cones_positions.json", 
}

class PublishMap(Node):

    cones = Marker()

    def __init__(self):
        super().__init__('publish_map')

        self.cones_positions_publisher = self.create_publisher(Marker,config["conesPositionsTopic"],1)
        self.get_logger().info("Publish map node initialized")
        self.init_cones_markers()
        self.export_cones()
        timer_period = 0.05     #20Hz
        self.timer = self.create_timer(timer_period,self.timer_callback)


    def init_cones_markers(self):
        #same as in ekf_odometry
        self.cones.header.frame_id = "track"
        self.cones.ns = "ConesAbsolutePos"
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
                

    def export_cones(self):
        with open(config["conesPositionsFilePath"]) as f_cones_pos:
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
            for x,y in orange_x,orange_y:
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
            for x,y in big_orange_x,big_orange_y:
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
               

    def timer_callback(self):
        self.cones_positions_publisher.publish(self.cones)
        self.get_logger().info("Published cones")


def main(args=None):
    rclpy.init(args=args)
    node = PublishMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
