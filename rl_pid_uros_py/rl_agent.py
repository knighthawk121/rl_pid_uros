
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import os
import json
from datetime import datetime
from collections import deque
import random


class DQNAgent:
    def __init__(self, state_dim=1, action_dim=1000, lr=1e-3, gamma=0.99, 
                 epsilon=1.0, epsilon_min=0.01, epsilon_decay=0.995,
                 memory_size=10000, batch_size=32, save_dir='dqn_data'):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.batch_size = batch_size
        self.save_dir = save_dir
        
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        # Define PID value ranges
        self.kp_range = np.linspace(0.1, 2.0, 10)
        self.ki_range = np.linspace(0.0, 0.5, 10)
        self.kd_range = np.linspace(0.0, 0.2, 10)
        
        # Neural Networks
        self.model = self._build_network()
        self.target_model = self._build_network()
        self.target_model.load_state_dict(self.model.state_dict())
        self.optimizer = optim.Adam(self.model.parameters(), lr=lr)
        
        # Experience replay
        self.memory = deque(maxlen=memory_size)
        
        # Training metrics
        self.episode_count = 0
        self.total_reward = 0
        self.best_reward = float('-inf')
        self.best_pid_values = None
        self.training_history = []
        
    def _build_network(self):
        return nn.Sequential(
            nn.Linear(self.state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, self.action_dim)
        )
        
    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        
    def get_pid_values(self, error):
        if np.random.random() < self.epsilon:
            kp_idx = np.random.randint(0, len(self.kp_range))
            ki_idx = np.random.randint(0, len(self.ki_range))
            kd_idx = np.random.randint(0, len(self.kd_range))
        else:
            state = torch.FloatTensor([error]).unsqueeze(0)
            q_values = self.model(state).detach().numpy()[0]
            action = np.argmax(q_values)
            kp_idx = action // 100
            ki_idx = (action % 100) // 10
            kd_idx = action % 10
            
        return (
            self.kp_range[kp_idx],
            self.ki_range[ki_idx],
            self.kd_range[kd_idx]
        )
        
    def replay(self):
        if len(self.memory) < self.batch_size:
            return
            
        batch = random.sample(self.memory, self.batch_size)
        states = torch.FloatTensor([x[0] for x in batch])
        actions = torch.LongTensor([x[1] for x in batch])
        rewards = torch.FloatTensor([x[2] for x in batch])
        next_states = torch.FloatTensor([x[3] for x in batch])
        dones = torch.FloatTensor([x[4] for x in batch])
        
        current_q = self.model(states).gather(1, actions.unsqueeze(1))
        next_q = self.target_model(next_states).max(1)[0].detach()
        target = rewards + (1 - dones) * self.gamma * next_q
        
        loss = nn.MSELoss()(current_q.squeeze(), target)
        
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
            
    def update_target_model(self):
        self.target_model.load_state_dict(self.model.state_dict())
        
    def save_model(self, reward=None):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_path = os.path.join(self.save_dir, f'dqn_model_{timestamp}.pth')
        
        if reward is not None:
            self.episode_count += 1
            self.total_reward += reward
            if reward > self.best_reward:
                self.best_reward = reward
                self.best_pid_values = self.get_best_pid_values()
                
            self.training_history.append({
                'episode': self.episode_count,
                'reward': float(reward),
                'epsilon': float(self.epsilon),
                'best_reward': float(self.best_reward)
            })
            
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'target_model_state_dict': self.target_model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'episode_count': self.episode_count,
            'total_reward': self.total_reward,
            'best_reward': self.best_reward,
            'best_pid_values': self.best_pid_values,
            'epsilon': self.epsilon
        }, file_path)
        
    def load_model(self, file_path):
        checkpoint = torch.load(file_path)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.target_model.load_state_dict(checkpoint['target_model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.episode_count = checkpoint['episode_count']
        self.total_reward = checkpoint['total_reward']
        self.best_reward = checkpoint['best_reward']
        self.best_pid_values = checkpoint['best_pid_values']
        self.epsilon = checkpoint['epsilon']




class SARSAAgent:
    def __init__(self, alpha=0.1, gamma=0.9, epsilon=0.1, save_dir='sarsa_data'):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.save_dir = save_dir
        
        # Create save directory if it doesn't exist
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        # Define PID value ranges (same as QLearningAgent)
        self.kp_range = np.linspace(0.1, 2.0, 10)
        self.ki_range = np.linspace(0.0, 0.5, 10)
        self.kd_range = np.linspace(0.0, 0.2, 10)
        
        # Initialize or load Q-table
        self.load_latest_q_table()
        
        # Training metrics
        self.episode_count = 0
        self.total_reward = 0
        self.best_reward = float('-inf')
        self.best_pid_values = None
        self.training_history = []
        
    def load_latest_q_table(self):
        try:
            files = [f for f in os.listdir(self.save_dir) if f.startswith('sarsa_q_table_') and f.endswith('.npz')]
            if files:
                latest_file = max(files)
                data = np.load(os.path.join(self.save_dir, latest_file))
                self.q_table = data['q_table']
                self.episode_count = int(data['episode_count'])
                self.best_reward = float(data['best_reward'])
                self.best_pid_values = data['best_pid_values']
            else:
                self._initialize_new_q_table()
        except:
            self._initialize_new_q_table()
            
    def _initialize_new_q_table(self):
        self.q_table = np.random.uniform(low=0.1, high=0.2, 
                                       size=(20, 10, 10, 10))  # state, kp, ki, kd
                                       
    def discretize_state(self, error):
        error_max = 180
        state_idx = int(np.clip((error + error_max) * 20 / (2 * error_max), 0, 19))
        return state_idx
        
    def get_pid_values(self, error):
        state = self.discretize_state(error)
        
        if np.random.uniform(0, 1) < self.epsilon:
            kp_idx = np.random.randint(0, len(self.kp_range))
            ki_idx = np.random.randint(0, len(self.ki_range))
            kd_idx = np.random.randint(0, len(self.kd_range))
        else:
            indices = np.unravel_index(np.argmax(self.q_table[state]), 
                                     self.q_table[state].shape)
            kp_idx, ki_idx, kd_idx = indices
            
        return (
            self.kp_range[kp_idx],
            self.ki_range[ki_idx],
            self.kd_range[kd_idx]
        )
        
    def update(self, state, pid_values, reward, next_error, next_pid_values):
        current_state = self.discretize_state(state)
        next_state = self.discretize_state(next_error)
        
        # Current PID indices
        kp_idx = np.argmin(np.abs(self.kp_range - pid_values[0]))
        ki_idx = np.argmin(np.abs(self.ki_range - pid_values[1]))
        kd_idx = np.argmin(np.abs(self.kd_range - pid_values[2]))
        
        # Next PID indices
        next_kp_idx = np.argmin(np.abs(self.kp_range - next_pid_values[0]))
        next_ki_idx = np.argmin(np.abs(self.ki_range - next_pid_values[1]))
        next_kd_idx = np.argmin(np.abs(self.kd_range - next_pid_values[2]))
        
        # SARSA update using next state-action pair
        current_q = self.q_table[current_state, kp_idx, ki_idx, kd_idx]
        next_q = self.q_table[next_state, next_kp_idx, next_ki_idx, next_kd_idx]
        
        # Update Q-value
        new_q = current_q + self.alpha * (reward + self.gamma * next_q - current_q)
        self.q_table[current_state, kp_idx, ki_idx, kd_idx] = new_q
        
        # Update metrics and save
        self.save_q_table(reward)
        self.epsilon = max(0.01, self.epsilon * 0.995)

    def save_q_table(self, reward=None):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_path = os.path.join(self.save_dir, f'sarsa_q_table_{timestamp}.npz')
        
        if reward is not None:
            self.episode_count += 1
            self.total_reward += reward
            if reward > self.best_reward:
                self.best_reward = reward
                self.best_pid_values = self.get_best_pid_values()
                
            self.training_history.append({
                'episode': self.episode_count,
                'reward': float(reward),
                'epsilon': float(self.epsilon),
                'best_reward': float(self.best_reward)
            })
            
        np.savez(file_path,
                 q_table=self.q_table,
                 episode_count=self.episode_count,
                 total_reward=self.total_reward,
                 best_reward=self.best_reward,
                 best_pid_values=self.best_pid_values)
                 
    def get_best_pid_values(self):
        best_values = np.unravel_index(np.argmax(np.max(self.q_table, axis=0)), 
                                     (10, 10, 10))
        return (
            self.kp_range[best_values[0]],
            self.ki_range[best_values[1]],
            self.kd_range[best_values[2]]
        )

