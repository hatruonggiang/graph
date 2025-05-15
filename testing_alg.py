import numpy as np
import argparse
from env import Robot, Package, Environment
from agent import Agents
from greedyagent import GreedyAgents as ga
from danh_visualizer import GridMapViewer

if __name__=="__main__":

    env = Environment(map_file = "map2.txt" , 
                      max_time_steps = 50 ,
                      n_robots = 1 , 
                      n_packages = 1 , 
                      seed = 2022)
    
    state = env.reset()
    # print("Initial State:", state)
    grids = []
    grids.append(env.render())
    
    why = Agents(env)
    path_1, acts_1 = why.astar(why.robots[0].position, why.packages[0].start, 0)
    path_2, acts_2 = why.astar(why.packages[0].start, why.packages[0].target, 1)
    for act in acts_1 + acts_2:
        env.step([act])
        grids.append(env.render())

    # agents = ga()   # You should define a default parameters here
    # agents.init_agents(state) # You have a change to init states which can be used or not. Depend on your choice
    # print(state['packages'])
    # print("Agents initialized.")
    
    # # Example actions for robots
    # list_actions = ['S', 'L', 'R', 'U', 'D']
    # n_robots = len(state['robots'])
    # done = False
    # t = 0
    # while not done:
    #     actions = agents.get_actions(state) 
    #     state, reward, done, infos = env.step(actions)
    
    #     grids.append(env.render())
    #     print(f"Reward: {reward}, Done: {done}, Infos: {infos}")
    #     print("Total Reward:", env.total_reward)
    #     print("Time step:", env.t)
    #     print("Packages:", state['packages'])
    #     print("Robots:", state['robots'])

    #     # For debug purpose
    #     t += 1
    #     if t == 100:
    #         break

    # grids = [env.get_state()['map']] 
    # # robots = [(1,1), (2,2), (3,4)]  
    # # for robot in robots:
    # #     env.add_robot(robot)
    # # grids.append(env.render())
    # print(len(grids))
    danh = GridMapViewer(grid_maps=grids)
    danh.run()

