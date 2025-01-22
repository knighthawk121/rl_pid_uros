import numpy as np
import os
import json
from datetime import datetime

class QLearningAgent:
    def __init__(self, alpha=0.1, gamma=0.9, epsilon=0.1, save_dir='q_learning_data'):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.save_dir = save_dir
        
        # Create save directory if it doesn't exist
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        # Define PID value ranges
        self.kp_range = np.linspace(0.1, 2.0, 10)
        self.ki_range = np.linspace(0.0, 0.5, 10)
        self.kd_range = np.linspace(0.0, 0.2, 10)
        
        # Try to load existing Q-table or create new one
        self.load_latest_q_table()
        
        # Track training metrics
        self.episode_count = 0
        self.total_reward = 0
        self.best_reward = float('-inf')
        self.best_pid_values = None
        
    def load_latest_q_table(self):
        """Load the most recent Q-table or create new one if none exists"""
        try:
            # Get list of all saved Q-tables
            files = [f for f in os.listdir(self.save_dir) if f.startswith('q_table_') and f.endswith('.npz')]
            
            if files:
                # Get most recent file
                latest_file = max(files)
                file_path = os.path.join(self.save_dir, latest_file)
                
                # Load Q-table and metadata
                data = np.load(file_path)
                self.q_table = data['q_table']
                self.episode_count = int(data['episode_count'])
                self.total_reward = float(data['total_reward'])
                self.best_reward = float(data['best_reward'])
                self.best_pid_values = data['best_pid_values']
                
                # Load training history
                history_file = file_path.replace('.npz', '_history.json')
                if os.path.exists(history_file):
                    with open(history_file, 'r') as f:
                        self.training_history = json.load(f)
                else:
                    self.training_history = []
                
                print(f"Loaded Q-table from {file_path}")
                print(f"Episodes completed: {self.episode_count}")
                print(f"Best reward achieved: {self.best_reward}")
                if self.best_pid_values is not None:
                    print(f"Best PID values: Kp={self.best_pid_values[0]:.4f}, "
                          f"Ki={self.best_pid_values[1]:.4f}, Kd={self.best_pid_values[2]:.4f}")
                
            else:
                self._initialize_new_q_table()
                
        except Exception as e:
            print(f"Error loading Q-table: {str(e)}")
            self._initialize_new_q_table()
            
    def _initialize_new_q_table(self):
        """Initialize a new Q-table with small random values"""
        self.q_table = np.random.uniform(low=0.1, high=0.2, 
                                       size=(20, 10, 10, 10))  # state, kp, ki, kd
        self.training_history = []
        print("Initialized new Q-table")
        
    def save_q_table(self, reward=None):
        """Save current Q-table and training metrics"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_path = os.path.join(self.save_dir, f'q_table_{timestamp}.npz')
        
        # Update metrics if reward provided
        if reward is not None:
            self.episode_count += 1
            self.total_reward += reward
            
            # Update best reward and PID values if current is best
            if reward > self.best_reward:
                self.best_reward = reward
                self.best_pid_values = self.get_best_pid_values()
                
            # Add to training history
            self.training_history.append({
                'episode': self.episode_count,
                'reward': float(reward),
                'epsilon': float(self.epsilon),
                'best_reward': float(self.best_reward)
            })
        
        # Save Q-table and metrics
        np.savez(file_path,
                 q_table=self.q_table,
                 episode_count=self.episode_count,
                 total_reward=self.total_reward,
                 best_reward=self.best_reward,
                 best_pid_values=self.best_pid_values)
        
        # Save training history separately as JSON
        history_file = file_path.replace('.npz', '_history.json')
        with open(history_file, 'w') as f:
            json.dump(self.training_history, f, indent=2)
            
        print(f"Saved Q-table to {file_path}")
    
    def get_best_pid_values(self):
        best_state = np.argmax(np.max(self.q_table, axis=(1,2,3)))
        best_values = np.unravel_index(np.argmax(self.q_table[best_state]), (10, 10, 10))
        return (
            self.kp_range[best_values[0]],
            self.ki_range[best_values[1]],
            self.kd_range[best_values[2]]
        )

    def discretize_state(self, error):
        """Convert continuous error to discrete state"""
        error_max = 180
        state_idx = int(np.clip((error + error_max) * 20 / (2 * error_max), 0, 19))
        return state_idx

    def get_pid_values(self, error):
        state = self.discretize_state(error)
        
        if np.random.uniform(0, 1) < self.epsilon:
            # Exploration: randomly select PID indices
            kp_idx = np.random.randint(0, len(self.kp_range))
            ki_idx = np.random.randint(0, len(self.ki_range))
            kd_idx = np.random.randint(0, len(self.kd_range))
        else:
            # Exploitation: select best PID indices from Q-table
            indices = np.unravel_index(
                np.argmax(self.q_table[state]), 
                self.q_table[state].shape
            )
            kp_idx, ki_idx, kd_idx = indices

        return (
            self.kp_range[kp_idx],
            self.ki_range[ki_idx],
            self.kd_range[kd_idx]
        )

    def update(self, state, pid_values, reward, next_error):
        """Update Q-table based on experience"""
        current_state = self.discretize_state(state)
        next_state = self.discretize_state(next_error)
        
        # Find indices of current PID values
        kp_idx = np.argmin(np.abs(self.kp_range - pid_values[0]))
        ki_idx = np.argmin(np.abs(self.ki_range - pid_values[1]))
        kd_idx = np.argmin(np.abs(self.kd_range - pid_values[2]))
        
        # Current Q-value
        current_q = self.q_table[current_state, kp_idx, ki_idx, kd_idx]
        
        # Maximum future Q-value
        next_max_q = np.max(self.q_table[next_state])
        
        # Update Q-value
        new_q = current_q + self.alpha * (reward + self.gamma * next_max_q - current_q)
        self.q_table[current_state, kp_idx, ki_idx, kd_idx] = new_q
        
        # Save Q-table after update
        self.save_q_table(reward)
        
        # Decay epsilon over time
        self.epsilon = max(0.01, self.epsilon * 0.995)  # Minimum 1% exploration