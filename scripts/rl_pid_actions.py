#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rl_pid_uros.action import TunePID, TunePIDmin
import numpy as np
from rl_pid_uros_py.q_learner import QLearningAgent
from rl_pid_uros_py.rl_agent import SARSAAgent, DQNAgent
import random
import asyncio
import sys
import torch
from collections import deque

class MotorPIDTuner(Node):
    def __init__(self):
        super().__init__('rl_pid_tuner')
        
        # System constraints
        self.POSITION_MIN = 0
        self.POSITION_MAX = 360
        self.MAX_VELOCITY = 50  # max allowable velocity
        self.SAFE_ACCELERATION = 20  # max safe acceleration
        
        # PID constraints
        self.KP_RANGE = (0.1, 2.0)
        self.KI_RANGE = (0.0, 0.5)
        self.KD_RANGE = (0.0, 0.2)
        
        # Performance parameters
        self.MAX_OVERSHOOT = 0.15  # 15% maximum overshoot
        self.SETTLING_THRESHOLD = 0.02  # 2% settling threshold
        self.MIN_SETTLING_TIME = 0.5  # minimum settling time in seconds
        
        # Anti-windup parameters
        self.integral_sum = 0.0
        self.integral_limit = 100.0  # anti-windup limit
        
        # Initialize target transition system
        self.current_target = 90  # start at midpoint
        self.target_queue = deque(maxlen=5)
        self.transition_rate = 0.2  # target change per second
        
        # Get agent selection from user
        self.agent = self.select_agent()

        self.callback_group = ReentrantCallbackGroup()
        self.action_client = self.select_platform()
        
        # Enhanced state tracking
        self.current_goal_handle = None
        self.goal_active = False
        self.latest_feedback = None
        self.feedback_history = []
        self.goal_timeout = 30.0
        self.performance_history = []
        
        # Conservative initial PID values
        self.initial_kp = 0.3
        self.initial_ki = 0.0
        self.initial_kd = 0.05

        # Exploration parameters
        self.exploration_phase = True
        self.exploration_episodes = 20
        self.episode_count = 0
        
        self._setup_logging()
        self._initialize_action_client()
        self._start_timer()
    
    def select_platform(self):
        while True: 
            print('\nSelect platform:')
            print('1.ROS2-Raspi Server')
            print('2. MicroROS - ESP32 server')

            try: 
                choice = input("Enter Choice (1 or 2):").strip()
                if choice == '1':
                    self.action_client = ActionClient(self,TunePID,'tune_pid_action',
                                callback_group=self.callback_group)
                    return self.action_client
                if choice == '2':
                    self.action_client = ActionClient(self,TunePIDmin,'tune_pid_action',
                                callback_group=self.callback_group)
                    return self.action_client
            except Exception as e:
                print(f"Error selecting platform: {str(e)}")
                sys.exit(1)

    def _setup_logging(self):
        """Enhanced logging setup"""
        self.get_logger().info(f'PID tuner initialized with {self.agent.episode_count} previous episodes')
        if hasattr(self.agent, 'best_pid_values') and self.agent.best_pid_values is not None:
            self.get_logger().info(
                f'Best known PID values - Kp: {self.agent.best_pid_values[0]:.4f}, '
                f'Ki: {self.agent.best_pid_values[1]:.4f}, '
                f'Kd: {self.agent.best_pid_values[2]:.4f}'
            )

    def _initialize_action_client(self):
        """Initialize and verify action client"""
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server is available!')

    def _start_timer(self):
        """Initialize timer with gradual target transitions"""
        self.timer = self.create_timer(10.0, self.send_new_goal_wrapper)
        self.get_logger().info('Timer started for sending goals every 10 seconds')

    def generate_next_target(self):
        """Generate smooth target transitions"""
        if not self.target_queue:
            new_target = random.randint(self.POSITION_MIN, self.POSITION_MAX)
            steps = np.linspace(self.current_target, new_target, 5)
            self.target_queue.extend(steps)
        return int(self.target_queue.popleft())

    def apply_anti_windup(self, error, dt):
        """Implement anti-windup mechanism"""
        self.integral_sum += error * dt
        self.integral_sum = np.clip(self.integral_sum, -self.integral_limit, self.integral_limit)
        return self.integral_sum

    def calculate_safe_pid_values(self, kp, ki, kd):
        """Ensure PID values are within safe bounds"""
        safe_kp = np.clip(kp, self.KP_RANGE[0], self.KP_RANGE[1])
        safe_ki = np.clip(ki, self.KI_RANGE[0], self.KI_RANGE[1])
        safe_kd = np.clip(kd, self.KD_RANGE[0], self.KD_RANGE[1])
        return safe_kp, safe_ki, safe_kd
    

    def select_agent(self):
        while True:
            print("\nSelect RL agent type:")
            print("1. Q-Learning")
            print("2. SARSA")
            print("3. DQN (Deep Q-Network)")
            
            try:
                choice = input("Enter choice (1-3): ").strip()
                if choice == "1":
                    return QLearningAgent()
                elif choice == "2":
                    return SARSAAgent()
                elif choice == "3":
                    return DQNAgent()
                else:
                    print("Invalid choice. Please select 1-3.")
            except Exception as e:
                print(f"Error selecting agent: {str(e)}")
                sys.exit(1)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.latest_feedback = feedback
        self.feedback_history.append({
            'error': feedback.current_error,
            'position': feedback.current_position,
            'target': feedback.target_position,
            'timestamp': self.get_clock().now().to_msg()
        })
        
        # Log every 5th feedback for monitoring
        if len(self.feedback_history) % 5 == 0:
            self.get_logger().info(
                f'Current Error: {feedback.current_error:.2f}, '
                f'Position: {feedback.current_position}, '
                f'Target: {feedback.target_position}'
            )

    async def send_new_goal_wrapper(self):
        if not self.goal_active:
            await self.send_new_goal()

    async def send_new_goal(self):
        """Enhanced goal sending with safety checks"""
        if self.goal_active:
            self.get_logger().warn('Previous goal still active, canceling...')
            if self.current_goal_handle:
                await self.current_goal_handle.cancel_goal_async()
            self.goal_active = False
            await asyncio.sleep(1.0)
            
        try:
            state = self.get_current_state()
            
            # Determine PID values based on exploration phase
            if self.exploration_phase and self.episode_count < self.exploration_episodes:
                kp = random.uniform(self.KP_RANGE[0], self.KP_RANGE[1])
                ki = random.uniform(self.KI_RANGE[0], self.KI_RANGE[1])
                kd = random.uniform(self.KD_RANGE[0], self.KD_RANGE[1])
                self.episode_count += 1
            else:
                if len(self.feedback_history) < 2:
                    kp, ki, kd = self.initial_kp, self.initial_ki, self.initial_kd
                else:
                    kp, ki, kd = self.agent.get_pid_values(state)
                    kp, ki, kd = self.calculate_safe_pid_values(kp, ki, kd)
            
            # Generate smooth target transition
            target_position = self.generate_next_target()
            self.current_target = target_position
            
            if self.select_platform == '1':
                goal_msg = TunePID.Goal()
                goal_msg.kp = float(kp)
                goal_msg.ki = float(ki)
                goal_msg.kd = float(kd)
                goal_msg.target_position = int(target_position)
            else: 
                goal_msg = TunePIDmin.Goal()
                goal_msg.kp = float(kp)
                goal_msg.ki = float(ki)
                goal_msg.kd = float(kd)
                goal_msg.target_position = int(target_position)

            self.get_logger().info(
                f'Sending new goal - kp: {kp:.4f}, ki: {ki:.4f}, kd: {kd:.4f}, '
                f'target: {target_position}'
            )
            
            # Reset tracking variables
            self.feedback_history = []
            self.integral_sum = 0.0
            
            # Send goal and handle response
            send_goal_future = await self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            
            if not send_goal_future.accepted:
                self.get_logger().error('Goal was rejected!')
                return

            self.current_goal_handle = send_goal_future
            self.goal_active = True
            
            # Handle goal result
            try:
                get_result_future = await send_goal_future.get_result_async()
                result = get_result_future.result
                
                if result:
                    self.get_logger().info(f'Goal succeeded with final error: {result.final_error}')
                    reward = self.calculate_episode_reward()
                    next_state = self.get_current_state()
                    
                    # Update agent based on type
                    self.update_agent(state, (kp, ki, kd), reward, next_state)
                    
                    # Store performance metrics
                    self.performance_history.append({
                        'final_error': result.final_error,
                        'settling_time': len(self.feedback_history) * 0.1,
                        'reward': reward,
                        'pid_values': (kp, ki, kd)
                    })
                    
            except asyncio.TimeoutError:
                self.get_logger().error('Goal timed out!')
                if self.current_goal_handle:
                    await self.current_goal_handle.cancel_goal_async()
                    
        except Exception as e:
            self.get_logger().error(f'Error in send_new_goal: {str(e)}')
        finally:
            self.goal_active = False

    def calculate_episode_reward(self):
        """Enhanced reward calculation with multiple performance metrics"""
        if not self.feedback_history:
            return -100.0
            
        errors = [abs(fb['error']) for fb in self.feedback_history]
        final_error = errors[-1]
        settling_time = len(errors) * 0.1
        
        # Calculate performance metrics
        overshoot = max(0, max(errors) / self.current_target - 1)
        settling_idx = next((i for i, e in enumerate(errors) if abs(e) <= self.SETTLING_THRESHOLD), len(errors))
        settling_time = settling_idx * 0.1
        
        # Calculate velocity and acceleration
        positions = [fb['position'] for fb in self.feedback_history]
        velocities = np.diff(positions) / 0.1
        accelerations = np.diff(velocities) / 0.1 if len(velocities) > 1 else [0]
        
        # Check various performance criteria
        steady_state_reached = all(abs(e) <= self.SETTLING_THRESHOLD for e in errors[-5:])
        max_time_exceeded = settling_time > 5.0
        velocity_violation = any(abs(v) > self.MAX_VELOCITY for v in velocities)
        acceleration_violation = any(abs(a) > self.SAFE_ACCELERATION for a in accelerations)
        overshoot_violation = overshoot > self.MAX_OVERSHOOT
        motor_stalled = len(velocities) > 5 and all(abs(v) < 1.0 for v in velocities[-5:])
        
        # Calculate reward components
        error_penalty = -0.5 * final_error
        time_penalty = -0.3 * settling_time
        stability_bonus = 50.0 if steady_state_reached else 0.0
        
        # Apply penalties for violations
        penalty = 0.0
        if max_time_exceeded: penalty -= 500.0
        if velocity_violation: penalty -= 1000.0
        if acceleration_violation: penalty -= 1000.0
        if overshoot_violation: penalty -= 500.0
        if motor_stalled and not steady_state_reached: penalty -= 1000.0
        
        reward = error_penalty + time_penalty + stability_bonus + penalty
        
        # Log detailed metrics
        self.get_logger().info(
            f'Episode metrics - Error: {final_error:.2f}, Time: {settling_time:.2f}s, '
            f'Overshoot: {overshoot*100:.1f}%, Stable: {steady_state_reached}, '
            f'Stalled: {motor_stalled}, Reward: {reward:.2f}'
        )
        
        return reward

    def get_current_state(self):
        """Enhanced state representation"""
        if self.latest_feedback is not None:
            # Include more state information for better decision making
            error = self.latest_feedback.current_error
            position = self.latest_feedback.current_position
            target = self.latest_feedback.target_position
            
            # Calculate additional state features
            error_change = 0.0
            if len(self.feedback_history) >= 2:
                prev_error = self.feedback_history[-2]['error']
                error_change = error - prev_error
            
            return np.array([error, error_change, position, target])
        return np.zeros(4)

    def update_agent(self, state, action, reward, next_state):
        """Update agent based on its type"""
        if isinstance(self.agent, SARSAAgent):
            next_pid_values = self.agent.get_pid_values(next_state)
            self.agent.update(state, action, reward, next_state, next_pid_values)
        elif isinstance(self.agent, DQNAgent):
            current_state = torch.FloatTensor([state])
            next_state_tensor = torch.FloatTensor([next_state])
            self.agent.remember(current_state.numpy(), action, reward, next_state_tensor.numpy(), True)
            self.agent.replay()
            if self.agent.episode_count % 10 == 0:
                self.agent.update_target_model()
        else:  # QLearningAgent
            self.agent.update(state, action, reward, next_state)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotorPIDTuner()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()   
        finally:
            executor.shutdown()
            node.destroy_node()

    except KeyboardInterrupt:
            print("Received keyboard interrupt, shutting down...")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()