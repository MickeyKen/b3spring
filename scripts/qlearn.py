# coding:utf-8

import gym
from gym import wrappers
import numpy as np
import time

def bins(clip_min, clip_max, num):
    return np.linspace(clip_min, clip_max, num + 1)[1:-1]

def digitize_state(observation):
    cart_pos, cart_v, pole_angle, pole_v = observation
    digitized = [
        np.digitize(cart_pos, bins=bins(-2.4, 2.4, num_dizitized)),
        np.digitize(cart_v, bins=bins(-3.0, 3.0, num_dizitized)),
        np.digitize(pole_angle, bins=bins(-0.5, 0.5, num_dizitized)),
        np.digitize(pole_v, bins=bins(-2.0, 2.0, num_dizitized))
    ]
    return sum([x * (num_dizitized**i) for i, x in enumerate(digitized)])


def get_action(next_state, episode):
    epsilon = 0.5 * (1 / (episode + 1))
    if epsilon <= np.random.uniform(0, 1):
        next_action = np.argmax(q_table[next_state])
    else:
        next_action = np.random.choice([0, 1])
    return next_action


def update_Qtable(q_table, state, action, reward, next_state):
    gamma = 0.99
    alpha = 0.5
    next_Max_Q=max(q_table[next_state][0],q_table[next_state][1] )
    q_table[state, action] = (1 - alpha) * q_table[state, action] +\
            alpha * (reward + gamma * next_Max_Q)

    return q_table

env = gym.make('CartPole-v0')
max_number_of_steps = 200 
num_consecutive_iterations = 100 
num_episodes = 2000 
goal_average_reward = 195 
num_dizitized = 6 
q_table = np.random.uniform(
    low=-1, high=1, size=(num_dizitized**4, env.action_space.n))

total_reward_vec = np.zeros(num_consecutive_iterations) 
final_x = np.zeros((num_episodes, 1)) 
islearned = 0 
isrender = 0 

for episode in range(num_episodes):  
    observation = env.reset()
    state = digitize_state(observation)
    action = np.argmax(q_table[state])
    episode_reward = 0
    
    for t in range(max_number_of_steps):  
        if islearned == 1:  
            env.render()
            time.sleep(0.1)
            print (observation[0])  
        env.render()
        observation, reward, done, info = env.step(action)

        if done:
            if t < 195:
                reward = -200 
            else:
                reward = 1
        else:
            reward = 1 

        episode_reward += reward 
        next_state = digitize_state(observation) 
        q_table = update_Qtable(q_table, state, action, reward, next_state)
 
        action = get_action(next_state, episode) 

        state = next_state

        if done:
            print('%d Episode finished after %f time steps / mean %f' %
                  (episode, t + 1, total_reward_vec.mean()))
            total_reward_vec = np.hstack((total_reward_vec[1:],
                                          episode_reward))  
            if islearned == 1:  
                final_x[episode, 0] = observation[0]
            break

    if (total_reward_vec.mean() >= goal_average_reward):  
        print('Episode %d train agent successfuly!' % episode)
        islearned = 1
        #np.savetxt('learned_Q_table.csv',q_table, delimiter=",")
        if isrender == 0:
            #env = wrappers.Monitor(env, './movie/cartpole-experiment-1')
            isrender = 1
    
    #if episode>10:
    #    if isrender == 0:
    #        env = wrappers.Monitor(env, './movie/cartpole-experiment-1')
    #        isrender = 1
    #    islearned=1;

if islearned:
    np.savetxt('final_x.csv', final_x, delimiter=",")
