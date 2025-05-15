import numpy as np
from env import Environment, Robot, Package
from scipy.optimize import linear_sum_assignment
import heapdict as hp

class Agents:

    def __init__(self, env: Environment):
        """
            TODO:
        """
        self.env = env
        
        self.packages = env.packages
        self.packages_assigned = [False] * len(self.packages)
        
        self.robots = env.robots
        self.temporary_path = []
        self.n_robots = len(self.robots)
        self.state = self.env.get_state()

        self.map = self.state['map']
        
        self.is_init = False

    def manhattan(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def euler(self, a, b):
        return (a[0] - b[0])**2 + (a[1] - b[1])**2
    
    def check_neighbors(self, a):
        neighbors = []
        for action in ['R', 'L', 'U', 'D']:
            neigh = self.env.compute_new_position(a, action)
            if self.env.is_free_cell(neigh):
                neighbors.append((neigh, action))
        if not neighbors:
            return None
        return neighbors

    def astar(self, start, goal, steitaus: int):
        """
            return path, actions
        """
        openList = {start: [0, 'S', None]}
        fringe = hp.heapdict()
        fringe[start] = self.manhattan(start, goal)
        closedList = {}

        while openList:
            node = fringe.popitem()[0]
            other = openList[node]
            closedList[node] = [other[0] + self.manhattan(node, goal), other[1], other[2]]

            if node == goal:
                path = []
                actions = []
                copy = node
                while closedList[copy][2] != None:
                    act = (closedList[copy][1], '0')
                    path.append(copy)
                    actions.append(act)
                    copy = closedList[copy][2]
                actions[1] = (actions[1][0], str(steitaus + 1))
                path.reverse()
                actions.reverse()
                print(actions)
                return path, actions

            for neighbor, action in self.check_neighbors(node):
                if closedList.get(neighbor) != None:
                    continue
                now = openList[node][0] + self.manhattan(node, goal)
                if (closedList.get(neighbor) != None):
                    if (closedList[neighbor][1] <=  now):
                        continue
                openList[neighbor] = [other[0] + 1, action, node]
                fringe[neighbor] = now

    def get_temp_paths(self):
        for i in range(self.n_robots):
            path_take, actions_take = self.astar(self.robots[i].position, self.packages[self.robots[i].carrying].start, 0)
            path_deli, actions_deli = self.astar(self.packages[self.robots[i].carrying].start, self.packages[self.robots[i].target].start, 1)
            self.temporary_path.append(actions_take + actions_deli)

    def package_assign(self):

        timestep, grid, robots, packages = self.env.get_state()

        available_packages = [
            pkg for pkg in packages
            if pkg[5] == 0 and timestep >= pkg[2]  # status=0 (chưa lấy), timestep >= start_time
        ]
        if not available_packages:
            print('no packages available at the time')
            return [] 

        available_robots = [
            i for i in range(self.num_agents)
            if robots[i][1] is None  # carrying is None
        ]
        if not available_robots:
            print('no robots available at the time')
            return []  # Không có robot khả dụng
        
        num_assignments = min(len(available_robots), len(available_packages))
        selected_packages = available_packages[:num_assignments]

        # Tính ma trận chi phí
        cost_matrix = np.zeros((len(available_robots), num_assignments))
        for i, robot_idx in enumerate(available_robots):
            robot_pos = robots[robot_idx].position
            for j, pkg in enumerate(selected_packages):
                pkg_start = pkg.start  
                pkg_target = pkg.target
                cost_matrix[i, j] = abs(robot_pos[0] - pkg_start[0]) + abs(robot_pos[1] - pkg_start[1]) + \
                                    abs(pkg_start[0] - pkg_target[0]) + abs(pkg_start[1] - pkg_start[1])

        # Chạy Hungarian Algorithm
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Tạo danh sách gán
        assignments = []
        for i, j in zip(row_ind, col_ind):
            robot_idx = available_robots[i]
            package_id = selected_packages[j].package_id  # package id
            self.robots[robot_idx].carrying = package_id
            assignments.append((robot_idx, package_id))

        return assignments

    def get_actions(self, state):
        """
            get actions for robots
        """
        t = state['Timestep']
        actions = []
        for i in range(self.n_robots):
            move = np.random.randint(0, len(list_actions))
            pkg_act = np.random.randint(0, 3)
            actions.append((list_actions[move], str(pkg_act)))

        return actions
