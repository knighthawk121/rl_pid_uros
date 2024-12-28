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
        # Map the state to PID indices
        kp_idx = int(np.clip(state % 10, 0, 9))  # Example: Use state modulo for kp index
        ki_idx = int(np.clip((state + 1) % 10, 0, 9))  # Increment for ki index
        kd_idx = int(np.clip((state + 2) % 10, 0, 9))  # Increment for kd index

        # Retrieve action from Q-table
        action = self.get_action(kp_idx, ki_idx, kd_idx)

        # Define how each action maps to PID value changes (decrease, stay, increase)
        kp = kp_idx * 0.1
        ki = ki_idx * 0.01
        kd = kd_idx * 0.001

        if action == 0:  # Decrease
            kp = max(0, kp - 0.1)
            ki = max(0, ki - 0.001)
            kd = max(0, kd - 0.0001)
        elif action == 2:  # Increase
            kp += 0.1
            ki += 0.001
            kd += 0.0001

        return kp, ki, kd
