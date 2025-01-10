import numpy as np

class QLearningAgent:
    def __init__(self, alpha=0.1, gamma=0.9, epsilon=0.1):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        
        # Define PID value ranges
        self.kp_range = np.linspace(0.1, 2.0, 10)  # 10 possible Kp values
        self.ki_range = np.linspace(0.0, 0.5, 10)  # 10 possible Ki values
        self.kd_range = np.linspace(0.0, 0.2, 10)  # 10 possible Kd values
        
        # Initialize Q-table with small random values instead of zeros
        self.q_table = np.random.uniform(low=0.1, high=0.2, 
                                       size=(20, 10, 10, 10))  # state, kp, ki, kd

    def discretize_state(self, error):
        """Convert continuous error to discrete state"""
        # Discretize error into 20 states (-180 to +180 degrees for position error)
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

        # Return actual PID values
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