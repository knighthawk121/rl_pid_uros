#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rl_pid_uros.action import TunePID
import RPi.GPIO as GPIO
import time
from threading import Lock
import math
import socket
import netifaces as ni
import asyncio

class NetworkConfig:
    @staticmethod
    def get_ip_address():
        # Get IP address of the wireless interface
        try:
            # Try wlan0 first (most common WiFi interface name)
            ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
        except (ValueError, KeyError):
            try:
                # Try wifi0 as fallback
                ip = ni.ifaddresses('wifi0')[ni.AF_INET][0]['addr']
            except (ValueError, KeyError):
                # If both fail, use hostname resolution
                hostname = socket.gethostname()
                ip = socket.gethostbyname(hostname)
        return ip

    @staticmethod
    def check_network_connection():
        try:
            # Try to connect to Google's DNS to check internet connectivity
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False

class MotorControl:
    def __init__(self):
        # GPIO pins for motor control
        self.ENC_A = 23  # Change these pins according to your setup
        self.ENC_B = 24
        self.PWM = 18
        self.IN1 = 20
        self.IN2 = 21
        self.STBY = 16

        # PID parameters
        self.position = 0
        self.target = 0
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.01
        self.eprev = 0
        self.eintegral = 0
        self.prev_time = time.time()
        self.goal_active = False
        
        # Constants
        self.MIN_DELTA_T = 0.001
        self.EPSILON = 1.0e-6
        self.DEADBAND = 5
        self.MAX_INTEGRAL = 100.0
        self.MAX_PWM_VALUE = 255
        
        self.position_lock = Lock()
        self.pid_lock = Lock()
        
        self.setup_gpio()

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.PWM, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.STBY, GPIO.OUT)
        
        # Setup PWM
        self.pwm = GPIO.PWM(self.PWM, 1000)  # 1000 Hz frequency
        self.pwm.start(0)
        
        # Setup encoder interrupt
        GPIO.add_event_detect(self.ENC_A, GPIO.RISING, callback=self.read_encoder)

    def read_encoder(self, channel):
        with self.position_lock:
            b = GPIO.input(self.ENC_B)
            self.position = self.position + 1 if b > 0 else self.position - 1

    def get_position(self):
        with self.position_lock:
            return self.position

    def set_motor(self, direction, pwm_value):
        GPIO.output(self.STBY, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(pwm_value)
        GPIO.output(self.IN1, GPIO.HIGH if direction == 1 else GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW if direction == 1 else GPIO.HIGH)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()


class PIDActionServer(Node):
    def __init__(self):
        super().__init__('pid_action_server')
        
        # Network configuration
        self.network_config = NetworkConfig()
        self.ip_address = self.network_config.get_ip_address()
        
        self.get_logger().info(f'Server IP address: {self.ip_address}')
        
        if not self.network_config.check_network_connection():
            self.get_logger().warning('No network connection detected!')
        
        self.motor = MotorControl()
        self.callback_group = ReentrantCallbackGroup()
        
        # Create the action server with QoS profile for reliable communication
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self._action_server = ActionServer(
            self,
            TunePID,
            'tune_pid_action',
            self.execute_callback,
            callback_group=self.callback_group,
            qos_profile=qos_profile
        )
            
        self.get_logger().info('PID Action Server has been started')
        self.get_logger().info(f'Listening on {self.ip_address}')

        # Create timer for motor control loop
        self.create_timer(0.02, self.motor_control_callback)  # 50Hz control loop
        
        # Create timer for network monitoring
        self.create_timer(5.0, self.check_network_status)  # Check every 5 seconds

    def check_network_status(self):
        if not self.network_config.check_network_connection():
            self.get_logger().warning('Network connection lost!')
        
    def motor_control_callback(self):
        if not self.motor.goal_active:
            return

        with self.motor.pid_lock:
            # Get current time and position
            curr_time = time.time()
            delta_t = curr_time - self.motor.prev_time
            
            if delta_t <= 0:
                self.get_logger().error('Invalid time delta')
                return
                
            delta_t = max(delta_t, self.motor.MIN_DELTA_T)
            self.motor.prev_time = curr_time

            current_pos = self.motor.get_position()
            error = self.motor.target - current_pos

            # PID calculations
            dedt = (error - self.motor.eprev) / (delta_t + self.motor.EPSILON)
            self.motor.eintegral += error * delta_t
            self.motor.eintegral = max(min(self.motor.eintegral, 
                                         self.motor.MAX_INTEGRAL), 
                                     -self.motor.MAX_INTEGRAL)

            # Calculate control signal
            pid_output = ((self.motor.kp * error) + 
                         (self.motor.ki * self.motor.eintegral) + 
                         (self.motor.kd * dedt))

            # Convert to motor commands
            power = min(abs(pid_output), self.motor.MAX_PWM_VALUE)
            direction = 1 if pid_output >= 0 else -1

            # Apply deadband
            if power < self.motor.DEADBAND:
                power = 0
                direction = 0

            # Update motor
            self.motor.set_motor(direction, power)
            self.motor.eprev = error

            return error, current_pos

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = TunePID.Feedback()
        result = TunePID.Result()

        # Update PID parameters
        with self.motor.pid_lock:
            self.motor.kp = goal_handle.request.kp
            self.motor.ki = goal_handle.request.ki
            self.motor.kd = goal_handle.request.kd
            self.motor.target = goal_handle.request.target_position
            self.motor.eprev = 0
            self.motor.eintegral = 0
            self.motor.goal_active = True

        stable_count = 0
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.motor.goal_active = False
                result.final_error = feedback_msg.current_error
                return result

            error, current_pos = self.motor_control_callback()
            
            # Update feedback
            feedback_msg.current_error = error
            feedback_msg.current_position = current_pos
            feedback_msg.target_position = self.motor.target
            goal_handle.publish_feedback(feedback_msg)

            # Check for goal completion
            if abs(error) < self.motor.DEADBAND:
                stable_count += 1
                if stable_count >= 5:
                    break
            else:
                stable_count = 0

            await asyncio.sleep(0.1)  # 10Hz feedback rate

        self.motor.goal_active = False
        result.final_error = error
        goal_handle.succeed()
        
        return result

    def cleanup(self):
        self.motor.cleanup()

def main(args=None):
    rclpy.init(args=args)
    
    # FastDDS-specific parameters
    params = [
        ('dds.initial_peers', ['127.0.0.1:7400']),
        ('dds.default_domain_id', 0),
    ]
    
    action_server = PIDActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.cleanup()
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()