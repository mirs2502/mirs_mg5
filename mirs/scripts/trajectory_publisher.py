#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        self.declare_parameter('frame_id', 'odom')
        self.frame_id = self.get_parameter('frame_id').value
        
        self.path_pub = self.create_publisher(Path, '/traveled_path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        
        self.last_pose = None
        self.min_distance = 0.05 # 5cm以上動いたら記録
        
        # ジャンプ判定用
        self.jump_candidate = None
        self.jump_count = 0

    def odom_callback(self, msg):
        current_pose = msg.pose.pose
        
        if self.last_pose is None:
            self.last_pose = current_pose
            return

        dist = self.calculate_distance(self.last_pose, current_pose)

        # 異常値フィルタ: 0.5m以上一気に飛んだらノイズの可能性
        if dist > 0.5:
            # 連続して同じ場所にいるかチェック
            if self.jump_candidate is None:
                self.jump_candidate = current_pose
                self.jump_count = 1
            else:
                # 候補位置との距離が近ければ（安定していれば）カウントアップ
                candidate_dist = self.calculate_distance(self.jump_candidate, current_pose)
                if candidate_dist < 0.1:
                    self.jump_count += 1
                else:
                    # 違う場所に飛んだらリセット
                    self.jump_candidate = current_pose
                    self.jump_count = 1
            
            # 5回連続（約0.5秒）安定したら、それを正とする
            if self.jump_count > 5:
                # self.get_logger().info('Jump accepted. Updating pose.')
                self.last_pose = current_pose
                self.jump_candidate = None
                self.jump_count = 0
            
            return # 今回は描画しない

        # 正常範囲内なら候補はリセット
        self.jump_candidate = None
        self.jump_count = 0

        # 一定距離動いたら記録
        if dist > self.min_distance:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.pose = current_pose
            
            self.path_msg.poses.append(pose_stamped)
            self.path_msg.header.stamp = msg.header.stamp
            
            self.path_pub.publish(self.path_msg)
            self.last_pose = current_pose

    def calculate_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return (dx**2 + dy**2)**0.5

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
