import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import csv
import time
from std_msgs.msg import Int32
import threading
from pynput import keyboard
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')

        self.declare_parameter('filename', 'waypoints.csv')
        self.declare_parameter('action_server_name', 'navigate_to_pose')
        waypoints_filename = self.get_parameter('filename').value
        action_server_name = self.get_parameter('action_server_name').value
#        print(waypoints_filename)

        self.id_publisher_ = self.create_publisher(Int32, '/navigation_manager/next_waypointID', 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'navigation_manager/waypoint_pose', 10)
        self.state_publisher_ = self.create_publisher(Int32, '/navigation_manager/state', 10)

        self._action_client = ActionClient(self, NavigateToPose, action_server_name)
        self.waypoints_data = self.load_waypoints_from_csv(waypoints_filename)
        self.current_waypoint_index = 0
        self._last_feedback_time = self.get_clock().now()
        
        # /odomからEKFのposeを取得
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # odomのポーズを保持する変数を初期化
        self.odom_pose = None
        # AMCLに/initialposeをpublish
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # gpsとmapのposeを提供してくれるノードに対して指示
        self.gps_pose_enable_publisher_ = self.create_publisher(Int32, '/navigation_manager/gps_pose_enable', 10)
        self.map_pose_enable_publisher_ = self.create_publisher(Int32, '/navigation_manager/map_pose_enable', 10)

        int_msg = Int32(data=self.current_waypoint_index)
        # ナビゲーション開始を伝える
        self.status_msg = Int32()
        self.pub_status(1)
        
        
    def load_waypoints_from_csv(self, filename):
        waypoints_data = []
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                pose_stamped_msg = PoseStamped()
                pose_stamped_msg.header.frame_id = 'map'  # Adjust the frame_id as needed
                pose_stamped_msg.pose.position.x = float(row[1])
                pose_stamped_msg.pose.position.y = float(row[2])
                pose_stamped_msg.pose.position.z = float(row[3])
                pose_stamped_msg.pose.orientation.x = float(row[4])
                pose_stamped_msg.pose.orientation.y = float(row[5])
                pose_stamped_msg.pose.orientation.z = float(row[6])
                pose_stamped_msg.pose.orientation.w = float(row[7])

                waypoint_data = {
                    "pose": pose_stamped_msg,
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

    def send_goal(self, waypoint_data):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint_data["pose"]
        # TEST WAIT
        time.sleep(0.3)
        
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Action server not available, waiting...')
            self.pub_status(5)
        
        self.get_logger().info('Sending waypoint...')
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        int_msg = Int32(data=self.current_waypoint_index)
        # idを送信
        self.id_publisher_.publish(int_msg)
        # navigation2にposeを送信
        self.pose_publisher_.publish(waypoint_data["pose"])
        # gps_pose_providerにフラグを送信
        gps_pose_enable_msg = Int32(data=waypoint_data["gps_pose_enable"])
        self.gps_pose_enable_publisher_.publish(gps_pose_enable_msg)
        # map_pose_providerにフラグを送信
        map_pose_enable_msg = Int32(data=waypoint_data["map_pose_enable"])
        self.map_pose_enable_publisher_.publish(map_pose_enable_msg)
        # ここでinit_pose_pubをチェックして、/initialposeにposeを送信
        if waypoint_data["init_pose_pub"] == 1 and self.odom_pose is not None:
            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
            initial_pose_msg.header.frame_id = 'map'  # 'map'フレームは適宜調整してください
            initial_pose_msg.pose.pose = self.odom_pose.pose.pose  # OdometryのPoseWithCovarianceからPoseにコピー
            initial_pose_msg.pose.covariance = self.odom_pose.pose.covariance  # 共分散情報もコピー
            self.initial_pose_publisher.publish(initial_pose_msg)

    def pub_status(self, call_id):
            self.status_msg.data = call_id
            self.state_publisher_.publish(self.status_msg)


    def feedback_callback(self, feedback_msg):
        current_time = self.get_clock().now()
        if (current_time - self._last_feedback_time).nanoseconds >= 3e9:
            self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.distance_remaining))
            self._last_feedback_time = current_time

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')

        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        
    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info('Goal completed with result: {0}'.format(result))

        self.next_waypoint_data = self.waypoints_data[self.current_waypoint_index]
        current_stop_flag = self.next_waypoint_data["stop_flag"]
        current_skip_flag = self.next_waypoint_data["skip_flag"]
        current_gps_flag  = self.next_waypoint_data["gps_pose_enable"]
        current_map_flag  = self.next_waypoint_data["map_pose_enable"]

        self.get_logger().info('skip_flag: %s' % current_skip_flag)
        self.get_logger().info('status: %s' % status)

        if current_stop_flag == 1 and current_stop_flag == 1:
            self.pub_status(103)

        if current_gps_flag == 0 and current_map_flag == 0:
            self.pub_status(104)
        
        # SUCCEEDED または ABORTEDかつskip == 1かつstop==0のときに次のwaypointを目指す
        if status == GoalStatus.STATUS_SUCCEEDED: 
            self.current_waypoint_index += 1
        elif status == GoalStatus.STATUS_ABORTED and current_skip_flag == 1 and current_stop_flag == 0:
            self.pub_status(101)
            self.current_waypoint_index += 1
        else:
            # 同じWaypointを目指し直す。インクリメントしないでセットするだけ。つまり何もしない。
            self.pub_status(102)
        
        if self.current_waypoint_index < len(self.waypoints_data):
            self.next_waypoint_data = self.waypoints_data[self.current_waypoint_index]

            # stop==1かつSUCCEEDEDのときに停止する。ABORTEDのときには止まらず次のWaypointを目指す。
            if current_stop_flag == 1 and status == GoalStatus.STATUS_SUCCEEDED:
                self.pub_status(20)
                self.get_logger().info('Press n key to resume navigation.')
                threading.Thread(target=self.wait_for_user_input).start()
            else:
                self.next_waypoint_data["pose"].header.stamp = self.get_clock().now().to_msg()
                self.send_goal(self.next_waypoint_data)
        else:
            self.get_logger().info('Arrived at the last waypoint. Navigation complete.')
            self.pub_status(10)


    def odom_callback(self, msg):
        self.odom_pose = msg
        #self.odom_pose = msg.pose
        
    def run(self):
        if self.waypoints_data:
            self.send_goal(self.waypoints_data[self.current_waypoint_index])

    def wait_for_user_input(self):
        self.key_pressed = False

        def on_press(key):
            if key == keyboard.KeyCode.from_char('n'):
                self.key_pressed = True
                return False  # Stop the listener

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

        print("'n' key detected!")
        self.next_waypoint_data["pose"].header.stamp = self.get_clock().now().to_msg()
        #self.send_goal(self.next_waypoint_data["pose"])
        self.send_goal(self.next_waypoint_data)

def main(args=None):
    rclpy.init(args=args)

    waypoint_sender = WaypointSender()
    try:
        waypoint_sender.run()
        rclpy.spin(waypoint_sender)
    except KeyboardInterrupt:
        print("Received KeyboardInterrupt, shutting down...")
        waypoint_sender.pub_status(100)

    finally:
        waypoint_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

