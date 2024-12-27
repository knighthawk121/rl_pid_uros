
#import torch
#import torch.nn as nn
#import torch.optim as optim
import numpy as np

class QLearningAgent:
    def __init__(self, alpha=0.1, gamma=0.9, epsilon=0.1):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.q_table = np.zeros((10, 10, 10))  # 3D Q-table for KP, KI, KD

    def get_action(self, kp_idx, ki_idx, kd_idx):
        if np.random.uniform(0, 1) < self.epsilon:  # Explore
            return np.random.choice([0, 1, 2])  # Change in PID values: Decrease, Stay, Increase
        else:  # Exploit
            return np.argmax(self.q_table[kp_idx, ki_idx, kd_idx])

    def update_q_table(self, kp_idx, ki_idx, kd_idx, reward):
        best_future_q = np.max(self.q_table[kp_idx, ki_idx, kd_idx])
        current_q = self.q_table[kp_idx, ki_idx, kd_idx]
        self.q_table[kp_idx, ki_idx, kd_idx] += self.alpha * (reward + self.gamma * best_future_q - current_q)

    def get_pid_values(self, state):
        # Get indices based on current state or use predefined values
        kp_idx = state % 10  # Placeholder logic to get an index for kp
        ki_idx = (state + 1) % 10  # Example: incremented for ki
        kd_idx = (state + 2) % 10  # Example: incremented for kd

        # Retrieve action from Q-table
        action = self.get_action(kp_idx, ki_idx, kd_idx)

        # Define how each action maps to PID values (for example, -1 = decrease, 0 = stay, 1 = increase)
        kp = kp_idx * 0.1  # Translate index to PID value (e.g., 0 -> 0.0, 9 -> 0.9)
        ki = ki_idx * 0.01
        kd = kd_idx * 0.001

        # Modify PID values based on the action
        if action == 0:  # Decrease
            kp -= 0.1
            ki -= 0.001
            kd -= 0.0001
        elif action == 2:  # Increase
            kp += 0.1
            ki += 0.001
            kd += 0.0001

        return kp, ki, kd

# SARSA agent
class SARSAAgent:
    def __init__(self, alpha=0.1, gamma=0.9, epsilon=0.1):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.q_table = np.zeros((10, 10, 10))  # 3D Q-table for KP, KI, KD

    def get_action(self, kp_idx, ki_idx, kd_idx):
        if np.random.uniform(0, 1) < self.epsilon:  # Explore
            return np.random.choice([0, 1, 2])  # Change in PID values: Decrease, Stay, Increase
        else:  # Exploit
            return np.argmax(self.q_table[kp_idx, ki_idx, kd_idx])

    def update_q_table(self, kp_idx, ki_idx, kd_idx, reward, next_kp_idx, next_ki_idx, next_kd_idx, next_action):
        current_q = self.q_table[kp_idx, ki_idx, kd_idx]
        next_q = self.q_table[next_kp_idx, next_ki_idx, next_kd_idx][next_action]
        self.q_table[kp_idx, ki_idx, kd_idx] += self.alpha * (reward + self.gamma * next_q - current_q)
'''
# PPO agent
class PPOAgent:
    def __init__(self, state_dim, action_dim, lr=1e-3, gamma=0.99, eps_clip=0.2):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.policy = self.build_network(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.policy_old = self.build_network(state_dim, action_dim)
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.mse_loss = nn.MSELoss()

    def build_network(self, input_dim, output_dim):
        return nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, output_dim),
            nn.Softmax(dim=-1)
        )

    def select_action(self, state):
        state = torch.FloatTensor(state).unsqueeze(0)
        with torch.no_grad():
            action_probs = self.policy_old(state)
        action = np.random.choice(len(action_probs[0]), p=action_probs.numpy()[0])
        return action

    def update(self, memory):
        # Simplified update function for demonstration
        # Normally, you would use batches and multiple epochs
        for state, action, reward, next_state, done in memory:
            state = torch.FloatTensor(state).unsqueeze(0)
            action = torch.tensor(action).unsqueeze(0)
            reward = torch.tensor(reward).unsqueeze(0)

            # Compute advantage
            with torch.no_grad():
                old_probs = self.policy_old(state)
                old_prob = old_probs.gather(1, action.unsqueeze(1)).squeeze(1)

            # Compute new probabilities
            new_probs = self.policy(state)
            new_prob = new_probs.gather(1, action.unsqueeze(1)).squeeze(1)

            # Compute ratio
            ratio = new_prob / old_prob

            # Compute surrogate loss
            advantage = reward - reward.mean()
            surr1 = ratio * advantage
            surr2 = torch.clamp(ratio, 1 - self.eps_clip, 1 + self.eps_clip) * advantage
            loss = -torch.min(surr1, surr2).mean()

            # Update policy
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        # Update old policy
        self.policy_old.load_state_dict(self.policy.state_dict())
'''