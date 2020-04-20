import random
# import gym
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam


from simulator import Simulator
from vehicle import Vehicle

import matplotlib.pyplot as plt

# from scores.score_logger import ScoreLogger

# ENV_NAME = "CartPole-v1"

GAMMA = 0.95
LEARNING_RATE = 0.001

MEMORY_SIZE = 1000000
BATCH_SIZE = 20

EXPLORATION_MAX = 1.0
EXPLORATION_MIN = 0.01
EXPLORATION_DECAY = 0.998

OBSERV_SPACE = 4
ACTION_SPACE = 5

POS = 0
ANG = 1
NOW = 0
FUTURE = 4


class DQNSolver:

    def __init__(self, observation_space, action_space):
        self.exploration_rate = EXPLORATION_MAX

        self.action_space = action_space
        self.memory = deque(maxlen=MEMORY_SIZE)

        self.model = Sequential()
        self.model.add(Dense(24, input_shape=(observation_space,), activation="relu"))
        self.model.add(Dense(24, activation="relu"))
        self.model.add(Dense(self.action_space, activation="linear"))
        self.model.compile(loss="mse", optimizer=Adam(lr=LEARNING_RATE))

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() < self.exploration_rate:
            return random.randrange(self.action_space)
        q_values = self.model.predict(state)
        return np.argmax(q_values[0])

    def experience_replay(self):
        if len(self.memory) < BATCH_SIZE:
            return
        batch = random.sample(self.memory, BATCH_SIZE)
        for state, action, reward, state_next, terminal in batch:
            q_update = reward
            if not terminal:
                q_update = (reward + GAMMA * np.amax(self.model.predict(state_next)[0]))
            q_values = self.model.predict(state)
            q_values[0][action] = q_update
            self.model.fit(state, q_values, verbose=0)
        self.exploration_rate *= EXPLORATION_DECAY
        self.exploration_rate = max(EXPLORATION_MIN, self.exploration_rate)


def main():
    # env = gym.make(ENV_NAME)
    # score_logger = ScoreLogger(ENV_NAME)
    dqn_solver = DQNSolver(OBSERV_SPACE, ACTION_SPACE)
    run = 0
    vehicle = Vehicle()
    while True:
        run += 1
        step = 0
        vehicle.reset()
        vehicle.generate_new_path()

        state = get_current_state(vehicle)
        state = np.reshape(state, [1, OBSERV_SPACE])

        while True:
            step += 1
            #env.render()
            action = dqn_solver.act(state)
            # state_next, reward, terminal, info = env.step(action)
            state_next, reward, terminal, _ = take_step(vehicle,action,step) 
            reward = reward if not terminal else -reward
            state_next = np.reshape(state_next, [1, OBSERV_SPACE])
            dqn_solver.remember(state, action, reward, state_next, terminal)
            state = state_next
            if terminal:
                print("Run: " + str(run) + ", exploration: " + str(dqn_solver.exploration_rate) + ", score: " + str(step))
                plot_result(vehicle)
                break
            dqn_solver.experience_replay()

def plot_result(vehicle):
    plt.ion()
    plt.plot(vehicle.path.x, vehicle.path.y)
    plt.plot(vehicle.logger.x,vehicle.logger.y)
    plt.axis('equal')
    plt.grid()
    plt.show()
    plt.pause(0.3)
    plt.close()

def get_current_state(vehicle):
    _, _, _, _, yref = vehicle.detect_and_plan()
    states = np.array([yref[POS,NOW],yref[POS,FUTURE],yref[ANG,NOW],yref[ANG,FUTURE]]) # current/future pos reference, current/future heading reference
    states = np.reshape(states, [1, OBSERV_SPACE])
    return states

def take_step(vehicle,action,step):
    HARD_LEFT = 0
    LEFT = 1 
    RIGHT = 3
    HARD_RIGHT = 4
    LATERAL_ERROR_TOL = 2.5 # [meters]
    END_STEP = 1200 # steps to hit the goal reward!
    END_REWARD = 10
    STEP_REWARD = 0.1

    str_ang = 0
    if action == HARD_LEFT:
        str_ang = np.deg2rad(15)
    elif action == LEFT:
        str_ang = np.deg2rad(5)
    elif action == RIGHT:
        str_ang = np.deg2rad(-5)
    elif action == HARD_RIGHT:
        str_ang = np.deg2rad(-15)

    vehicle.move(str_ang)
    state_next = get_current_state(vehicle)

    reward = STEP_REWARD
    terminal = False
    if step > END_STEP: # Hit end of the road!
        terminal = True
        reward = -END_REWARD # This gets multiplied by -1 to become +1 reward in main loop
    if np.abs(state_next[POS,NOW]) > LATERAL_ERROR_TOL: # Deviated from path
        terminal = True
        reward = END_REWARD # This gets multiplied by -1 to become -1 reward in main loop
    info = []
    return state_next, reward, terminal, info

if __name__ == "__main__":
    main()