#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from tf_transformations import euler_from_quaternion

class MoveAndRotate(Node):
    def __init__(self):
        super().__init__('move_and_rotate')
        
        # ==========================================
        # 【設定変更エリア】
        # ここで前進距離と旋回角度を設定してください
        # ==========================================
        self.target_distance = 1.3        # 目標前進距離 [メートル]
        self.target_angle_deg = 60.0     # 目標旋回角度 [度] (360 + 45 = 405)
        # ==========================================
        
        # 度をラジアンに変換
        self.target_angle_rad = math.radians(self.target_angle_deg)
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # 状態管理フラグ
        # 0: 初期化待ち, 1: 前進中, 2: 旋回待機中, 3: 旋回中, 4: 終了
        self.state = 0
        
        # 直線移動用変数 (odom_linear_test.py より)
        self.start_x = None
        self.start_y = None
        self.current_distance = 0.0
        
        # 回転用変数 (odom_rotate_test.py より)
        self.last_yaw = None
        self.accumulated_angle = 0.0
        
        # 制御ループ
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Move and Rotate Node Started')
        self.get_logger().info(f'Plan: Move {self.target_distance}m then Rotate {self.target_angle_deg}deg')
        self.get_logger().info('Waiting for /odom data...')

    def get_yaw(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def odom_callback(self, msg):
        # 現在位置と姿勢の取得
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_yaw = self.get_yaw(msg)
        
        # 初期位置の設定 (初回のみ)
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.last_yaw = current_yaw
            self.state = 1 # 前進開始
            self.get_logger().info(f'Start Position: x={x:.3f}, y={y:.3f}')
            self.get_logger().info('--- Phase 1: Moving Forward ---')
            return

        # 1. 距離の計算 (常に更新)
        dx = x - self.start_x
        dy = y - self.start_y
        self.current_distance = math.sqrt(dx*dx + dy*dy)
        
        # 2. 角度の計算 (常に更新)
        # 角度の差分を計算 (ラップアラウンド処理)
        delta = current_yaw - self.last_yaw
        if delta < -math.pi:
            delta += 2 * math.pi
        elif delta > math.pi:
            delta -= 2 * math.pi
            
        # 旋回フェーズ中のみ角度を積算する
        if self.state == 3:
            self.accumulated_angle += delta
            
        self.last_yaw = current_yaw

    def control_loop(self):
        twist = Twist()
        
        # --- フェーズ1: 前進 ---
        if self.state == 1:
            remaining_distance = self.target_distance - self.current_distance
            
            if remaining_distance <= 0:
                # 目標到達
                twist.linear.x = 0.0
                self.publisher_.publish(twist)
                self.get_logger().info(f'Linear Target Reached! Distance: {self.current_distance:.3f} m')
                
                # 次のフェーズへの移行準備
                self.state = 2
                self.wait_start_time = time.time()
            else:
                # P制御的な速度調整 (最大0.3m/s, 最小0.05m/s)
                speed = 0.5 * remaining_distance
                speed = max(0.05, min(0.3, speed))
                
                twist.linear.x = speed
                self.publisher_.publish(twist)
                self.get_logger().info(f'[Move] Dist: {self.current_distance:.2f}/{self.target_distance:.1f} m (Vel: {speed:.2f})')

        # --- フェーズ2: 待機 (1秒停止) ---
        elif self.state == 2:
            if time.time() - self.wait_start_time > 1.0:
                self.state = 3
                self.accumulated_angle = 0.0 # 旋回角度の積算をリセット
                self.get_logger().info('--- Phase 2: Rotating ---')

        # --- フェーズ3: 旋回 ---
        elif self.state == 3:
            # 絶対値で比較
            remaining_angle = self.target_angle_rad - abs(self.accumulated_angle)
            
            if remaining_angle <= 0:
                # 目標到達
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                final_deg = math.degrees(self.accumulated_angle)
                self.get_logger().info(f'Rotate Target Reached! Final Angle: {final_deg:.3f} deg')
                
                self.state = 4
                time.sleep(1.0)
                raise SystemExit
            else:
                # P制御的な速度調整 (最大0.5rad/s, 最小0.1rad/s)
                speed = 0.5 * remaining_angle
                speed = max(0.1, min(0.5, speed))
                
                # 左回転(反時計回り)
                twist.angular.z = speed
                self.publisher_.publish(twist)
                
                current_deg = math.degrees(self.accumulated_angle)
                self.get_logger().info(f'[Rotate] Ang: {current_deg:.1f}/{self.target_angle_deg:.1f} deg (Vel: {speed:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = MoveAndRotate()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        print(e)
    finally:
        # 安全のため停止コマンドを送る
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
