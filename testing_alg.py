import numpy as np
import argparse
from env import Robot, Package, Environment
from agent import Agents
import greedyagent
from danh_visualizer import GridMapViewer

if __name__=="__main__":

    env = Environment(map_file = "map1.txt" , 
                      max_time_steps = 20 ,
                      n_robots = 3 , 
                      n_packages = 1 , 
                      seed = 2025)

    grids = [env.grid] 
    # robots = [(1,1), (2,2), (3,4)]  
    # for robot in robots:
    #     env.add_robot(robot)
    grids.append(env.render())
    danh = GridMapViewer(grid_maps=grids)
    danh.run()

