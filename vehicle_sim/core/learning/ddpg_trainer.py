import numpy as np
import random
import torch
import matplotlib.pyplot as plt
from .ddpg_agent import Agent
from collections import deque
from vehicle_sim.core.vehicle import Vehicle


OBSERV_SPACE = 4
ACTION_SPACE = 5

POS = 0
ANG = 1
NOW = 0
FUTURE = 4

# number of agents
num_agents = 1
print('Number of agents:', num_agents)

# size of each action
action_size = 1
print('Size of each action:', action_size)

# examine the state space
states = np.zeros([1,OBSERV_SPACE])
state_size = states.shape[1]
print('There are {} agents. Each observes a state with length: {}'.format(states.shape[0], state_size))
print('The state for the first agent looks like:', states[0])

agent = Agent(state_size=state_size, action_size=action_size, random_seed=1)

def get_current_state(vehicle):
    _, _, _, _, yref = vehicle.detect_and_plan()
    states = np.array([yref[POS,NOW],yref[POS,FUTURE],yref[ANG,NOW],yref[ANG,FUTURE]]) # current/future pos reference, current/future heading reference
    states = np.reshape(states, [1, OBSERV_SPACE])
    return states

def take_step(vehicle,action,step):
    LATERAL_ERROR_TOL = 1 # [meters]
    END_STEP = 1200 # steps to hit the goal reward!
    END_REWARD = 10
    STEP_REWARD = 0.1

    vehicle.move(action)
    state_next = get_current_state(vehicle)

    terminal = False
    reward = STEP_REWARD
    reward_factor = 0
    if state_next[0,0] < LATERAL_ERROR_TOL / 2:
        reward_factor += 1/2
    if state_next[0,2] < np.abs(np.deg2rad(1)):
        reward_factor += 1/2
        # if state_next[0,2] < np.abs(np.deg2rad(1)):
        #     reward_factor += 1/4
    reward = reward * reward_factor

    if step > END_STEP: # Hit end of the road!
        terminal = True
        reward = END_REWARD # This gets multiplied by -1 to become +1 reward in main loop
    if np.abs(state_next[POS,NOW]) > LATERAL_ERROR_TOL: # Deviated from path
        terminal = True
        reward = -END_REWARD # This gets multiplied by -1 to become -1 reward in main loop
    info = []
    return state_next, np.reshape(reward,[1]), np.reshape(terminal,[1]), info

def plot_result(vehicle):
    plt.ion()
    plt.plot(vehicle.path.x, vehicle.path.y)
    plt.plot(vehicle.logger.x,vehicle.logger.y)
    plt.axis('equal')
    plt.grid()
    plt.show()
    plt.pause(0.3)
    plt.close()

def train(n_episodes=2000, max_t=20000):

    scores_deque = deque(maxlen=100)
    total_scores = []

    vehicle = Vehicle()

    for i_episode in range(1, n_episodes+1):
        vehicle.reset()
        vehicle.generate_new_path()

        states = get_current_state(vehicle)
        agent.reset()

        scores = np.zeros(num_agents)                          # initialize the score (for each agent)
        step = 0
        while True:
            action = agent.act(states)
            next_states, rewards, terminal, _ = take_step(vehicle,action,step) 
            agent.step(states, action, rewards, next_states, terminal)
            scores += rewards                                  # update the score (for each agent)
            states = next_states                               # roll over states to next time step
            if terminal:                                       # exit loop if episode finished
                # plot_result(vehicle)
                break
            step += 1
        scores_deque.append(np.mean(scores))
        total_scores.append(np.mean(scores))

        print('\rEpisode: \t{} \tScore: \t{:.2f} \tAverage Score: \t{:.2f}'.format(i_episode, np.mean(scores), np.mean(scores_deque)), end="")

        if i_episode % 100 == 0:
            torch.save(agent.actor_local.state_dict(), 'checkpoint_actor.pth')
            torch.save(agent.critic_local.state_dict(), 'checkpoint_critic.pth')

        if np.mean(list(scores_deque)[-5:])>=100.0:  # consider done when the average score reaches 30 or more
            print('\nEnvironment solved in {:d} episodes!\tAverage Score: {:.2f}'.format(i_episode-100, np.mean(scores_deque)))
            torch.save(agent.actor_local.state_dict(), 'checkpoint_actor.pth')
            torch.save(agent.critic_local.state_dict(), 'checkpoint_critic.pth')
            break

 
    plt.plot(np.arange(1, len(total_scores)+1), total_scores)
    plt.ylabel('Score')
    plt.xlabel('Episode #')
    plt.show()






