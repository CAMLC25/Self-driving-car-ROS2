#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os # Thêm thư viện để gọi lệnh dịch chuyển xe

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.br = CvBridge()
        
        # --- BIẾN TRẠNG THÁI ---
        self.prev_error = 0.0
        self.last_steering = 0.0
        self.is_finished = False
        self.start_time = time.time()

        # --- THÔNG SỐ TỐC ĐỘ CAO (3.5 m/s) & PD CHỐNG RUNG ---
        self.kp = 0.0035         
        self.kd = 0.0095         
        self.max_speed = 3.5     
        self.turn_speed = 2.5    
        self.approach_speed = 2.2 
        self.stop_area = 1000    

        # Khởi tạo Dashboard 4 mắt
        cv2.namedWindow("1-Telemetry", cv2.WINDOW_NORMAL)
        cv2.namedWindow("2-Mat_Na_Vang", cv2.WINDOW_NORMAL)
        cv2.namedWindow("3-Mat_Na_Do", cv2.WINDOW_NORMAL)
        cv2.namedWindow("4-Vung_Quet_ROI", cv2.WINDOW_NORMAL)

        self.get_logger().info("=== HE THONG SAN SANG: NHAN PHIM 'R' TREN DASHBOARD DE CHAY LAI ===")

    def image_callback(self, msg):
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        except: return

        height, width, _ = cv_image.shape
        hsv_full = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        twist = Twist()
        direction = "SEARCHING"
        error_val = self.prev_error

        # --- 1. NHẬN DIỆN VẠCH ĐỎ (DỪNG & RÀ PHANH) ---
        mask_red_full = cv2.inRange(hsv_full, np.array([0, 150, 100]), np.array([10, 255, 255])) + \
                        cv2.inRange(hsv_full, np.array([170, 150, 100]), np.array([180, 255, 255]))
        
        red_look_ahead = mask_red_full[int(height*0.4):height, :]
        red_area_total = cv2.countNonZero(red_look_ahead)

        stop_line_y = int(height * 0.95)
        red_bumper_roi = mask_red_full[stop_line_y:height, :]
        red_area_at_bumper = cv2.countNonZero(red_bumper_roi)

        # --- 2. NHẬN DIỆN VẠCH VÀNG (DÒ ĐƯỜNG) ---
        roi_top, roi_bottom = int(height * 0.5), int(height * 0.85)
        roi_img = cv_image[roi_top:roi_bottom, :]
        mask_yellow = cv2.inRange(hsv_full[roi_top:roi_bottom, :], np.array([18, 80, 80]), np.array([35, 255, 255]))
        M = cv2.moments(mask_yellow)

        # --- 3. LOGIC ĐIỀU KHIỂN ---
        if self.is_finished or red_area_at_bumper > self.stop_area:
            self.is_finished = True
            direction = "!!! DA DEN DICH !!!"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
        
        elif M['m00'] > 300:
            cx = int(M['m10']/M['m00'])
            error_val = float(cx - width / 2)
            
            steering = -(self.kp * error_val + self.kd * (error_val - self.prev_error))
            twist.angular.z = max(min(steering, 0.5), -0.5)
            
            # --- THUẬT TOÁN ĐIỀU TỐC THÔNG MINH (Adaptive Speed) ---
            if red_area_total > 5000:
                twist.linear.x = self.approach_speed
                direction = "RA PHANH VE DICH"
            elif abs(error_val) > 15:
                twist.linear.x = self.turn_speed
                direction = "RE TRAI <--" if twist.angular.z > 0 else "RE PHAI -->"
            else:
                twist.linear.x = self.max_speed
                direction = "DI THANG NHANH ^"

            self.last_steering = twist.angular.z
            self.prev_error = error_val
            cv2.circle(cv_image, (cx, roi_top + int(M['m01']/M['m00'])), 12, (0, 255, 0), -1)
            self.publisher.publish(twist)
        else:
            direction = "MAT VACH - BO NHO"
            twist.linear.x = 0.5
            twist.angular.z = self.last_steering
            self.publisher.publish(twist)

        # --- 4. HIỂN THỊ DASHBOARD ---
        cv2.line(cv_image, (int(width/2), 0), (int(width/2), height), (255, 0, 0), 2)
        cv2.line(roi_img, (int(width/2), 0), (int(width/2), roi_bottom - roi_top), (255, 0, 0), 2)
        cv2.rectangle(cv_image, (0, roi_top), (width, roi_bottom), (0, 255, 255), 1) 
        cv2.rectangle(cv_image, (0, stop_line_y), (width, height), (0, 0, 255), 2) 

        status_color = (0, 0, 255) if self.is_finished else (0, 255, 0)
        cv2.putText(cv_image, f"STATE: {direction}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.putText(cv_image, f"ERROR: {error_val:.1f}px", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(cv_image, f"BUMPER RED: {red_area_at_bumper}", (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(cv_image, f"SPEED: {twist.linear.x:.2f} m/s", (10, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("1-Telemetry", cv_image)
        cv2.imshow("2-Mat_Na_Vang", mask_yellow)
        cv2.imshow("3-Mat_Na_Do", mask_red_full)
        cv2.imshow("4-Vung_Quet_ROI", roi_img)
        
        # --- 5. LOGIC NUT BAM RESTART (NHAN 'R' DE CHAY LAI) ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r') or key == ord('R'):
            self.restart_robot()

    def restart_robot(self):
        self.get_logger().info("!!! DANG QUAY LAI DIEM XUAT PHAT... !!!")
        # Reset các biến trạng thái
        self.is_finished = False
        self.prev_error = 0.0
        self.last_steering = 0.0

        # Tọa độ ní yêu cầu: x=47.0, y=-2.4, z=0.1, Yaw=2.5 rad
        # Quaternion cho Yaw 2.5 rad: z=sin(2.5/2)=0.949, w=cos(2.5/2)=0.315
        
        # Lệnh Teleport xe về tọa độ xuất phát (x=30, y=-2.14, yaw=3.14)
        teleport_cmd = "gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 300 " \
                       "--req 'name: \"my_bot\", position: {x: 47.0, y: -2.4, z: 0.08}, orientation: {z: .949, w: 0.315}'"
        os.system(teleport_cmd)

    def stop_vehicle(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = 0.0, 0.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()