import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Pose,PoseArray
from sensor_msgs.msg import NavSatFix
import csv


class WaypointViewer(Node):
    def __init__(self):
        super().__init__('waypoint_viewer')

        self.declare_parameter('filename_list', ['waypoints.csv'])
        # self.declare_parameter('filename_list', ['waypoints.csv'])
        waypoints_filename_list = self.get_parameter('filename_list').value

        self.declare_parameter('datum_position', [35.681236 , 139.767125])
        self.datum_cord = self.get_parameter('datum_position').value
        
        self.pose_publisher_list = []
        self.waypoints_data_list = []

        for i in range(len(waypoints_filename_list)) :
            self.waypoints_data_list.append( self.load_waypoints_from_csv(waypoints_filename_list[i]) )
            self.pose_publisher_list.append( [self.create_publisher(PoseArray , 'navigation_manager/waypoint_poses/list'+ str(i) +'/gps' , 10) ,
                                              self.create_publisher(PoseArray , 'navigation_manager/waypoint_poses/list'+ str(i) +'/amcl', 10) ,
                                              self.create_publisher(PoseArray , 'navigation_manager/waypoint_poses/list'+ str(i) +'/initialpose', 10)
            ])

        self.fix_publisher_ = ( self.create_publisher(NavSatFix, 'fix' ,10))
        
        
    def load_waypoints_from_csv(self, filename):
        self.get_logger().info('load csv : %s'% filename)
        
        waypoints_data = []
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                pose_msg = Pose()
                # pose_msg.header.frame_id = 'map'  # Adjust the frame_id as needed
                pose_msg.position.x = float(row[1])
                pose_msg.position.y = float(row[2])
                pose_msg.position.z = float(row[3])
                pose_msg.orientation.x = float(row[4])
                pose_msg.orientation.y = float(row[5])
                pose_msg.orientation.z = float(row[6])
                pose_msg.orientation.w = float(row[7])

                waypoint_data = {
                    "pose": pose_msg,
                    "xy_goal_tol": float(row[8]),
                    "des_lin_vel": float(row[9]),
                    "stop_flag": int(row[10]),
                    "skip_flag": int(row[11]),
                    "gps_pose_enable": int(row[12]),
                    "map_pose_enable": int(row[13]),
                    "init_pose_pub": int(row[14])
                }

                waypoints_data.append(waypoint_data)

        return waypoints_data
    
    def publish_waypoints(self) :
        list_itr = 0
        for csv_data in self.waypoints_data_list :
            poses_gps = []
            poses_amcl = []
            poses_initialpose = []
            
            for waypoint in csv_data:
                if waypoint['gps_pose_enable'] == 1 :
                    poses_gps.append(waypoint['pose'])
                if waypoint['map_pose_enable'] == 1 :
                    poses_amcl.append(waypoint['pose'])
                if waypoint['init_pose_pub'] == 1 :
                    poses_initialpose.append(waypoint['pose'])
            
            pose_array_msg = PoseArray()
            pose_array_msg.header.stamp = self.get_clock().now().to_msg()
            pose_array_msg.header.frame_id = 'map'
            
            pose_array_msg.poses = poses_gps
            self.pose_publisher_list[list_itr][0].publish(pose_array_msg)

            pose_array_msg.poses = poses_amcl
            self.pose_publisher_list[list_itr][1].publish(pose_array_msg)

            pose_array_msg.poses = poses_initialpose
            self.pose_publisher_list[list_itr][2].publish(pose_array_msg)
            
            list_itr +=1

    def publish_fix(self):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.stamp    = self.get_clock().now().to_msg()
        navsatfix_msg.header.frame_id = 'map'
        navsatfix_msg.latitude        = self.datum_cord[0]
        navsatfix_msg.longitude       = self.datum_cord[1]
        
        self.fix_publisher_.publish(navsatfix_msg)
    
    def run(self):
        self.publish_waypoints()
        self.publish_fix()

def main(args=None):
    rclpy.init(args=args)

    waypoint_viewer = WaypointViewer()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(waypoint_viewer, ), daemon=True)
    thread.start()

    rate = waypoint_viewer.create_rate(1)
    
    try:
        while rclpy.ok():
            waypoint_viewer.run()
            rate.sleep()
            # rclpy.spin(waypoint_viewer)
    except KeyboardInterrupt:
        print("Received KeyboardInterrupt, shutting down...")
    finally:
        waypoint_viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

