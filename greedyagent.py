import numpy as np
import heapq
from scipy.optimize import linear_sum_assignment
# Run a BFS to find the path from start to goal
def run_bfs(map, start, goal):
    n_rows = len(map)
    n_cols = len(map[0])

    queue = []
    visited = set()
    queue.append((goal, []))
    visited.add(goal)
    d = {}
    d[goal] = 0

    while queue:
        current, path = queue.pop(0)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            next_pos = (current[0] + dx, current[1] + dy)
            if next_pos[0] < 0 or next_pos[0] >= n_rows or next_pos[1] < 0 or next_pos[1] >= n_cols:
                continue
            if next_pos not in visited and map[next_pos[0]][next_pos[1]] == 0:
                visited.add(next_pos)
                d[next_pos] = d[current] + 1
                queue.append((next_pos, path + [next_pos]))

    if start not in d:
        return 'S', 100000
    
    t = 0
    actions = ['U', 'D', 'L', 'R']
    current = start
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        next_pos = (current[0] + dx, current[1] + dy)
        if next_pos in d:
            if d[next_pos] == d[current] - 1:
                return actions[t], d[next_pos]
        t += 1
    return 'S', d[start]

def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

class GreedyAgents:

    def __init__(self):
        self.agents = []
        self.packages = []
        self.packages_free = []
        self.n_robots = 0
        self.state = None

        self.is_init = False

    def init_agents(self, state):
        self.state = state
        self.n_robots = len(state['robots'])
        self.map = state['map']
        self.robots = [(robot[0]-1, robot[1]-1, 0) for robot in state['robots']]
        self.robots_target = ['free'] * self.n_robots
        self.packages += [(p[0], p[1]-1, p[2]-1, p[3]-1, p[4]-1, p[5]) for p in state['packages']]

        self.packages_free = [True] * len(self.packages)

    def update_move_to_target(self, robot_id, target_package_id, phase='start'):

        if phase == 'start':
            distance = abs(self.packages[target_package_id][1]-self.robots[robot_id][0]) + \
            abs(self.packages[target_package_id][2]-self.robots[robot_id][1])
        else:
            # Switch to the distance to target (3, 4) if phase == 'target'
            distance = abs(self.packages[target_package_id][3]-self.robots[robot_id][0]) + \
            abs(self.packages[target_package_id][4]-self.robots[robot_id][1])
        i = robot_id
        #print(self.robots[i], distance, phase)

        # Step 4: Move to the package
        pkg_act = 0
        move = 'S'
        if distance >= 1:
            pkg = self.packages[target_package_id]
            
            target_p = (pkg[1], pkg[2])
            if phase == 'target':
                target_p = (pkg[3], pkg[4])
            move, distance = run_bfs(self.map, (self.robots[i][0], self.robots[i][1]), target_p)

            if distance == 0:
                if phase == 'start':
                    pkg_act = 1 # Pickup
                else:
                    pkg_act = 2 # Drop
        else:
            move = 'S'
            pkg_act = 1    
            if phase == 'start':
                pkg_act = 1 # Pickup
            else:
                pkg_act = 2 # Drop    

        return move, str(pkg_act)
    
    
    def package_assign(self):

        timestep, grid, robots, packages = self.env.get_state()

        available_packages = [
            pkg for pkg in packages
            if pkg[5] == 0 and timestep >= pkg[2]  # status=0 (chưa lấy), timestep >= start_time
        ]
        if not available_packages:
            print('no packages available at the time')
            return []  # Không có gói hàng để gán

        # Lấy danh sách robot khả dụng (không mang gói hàng)
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
            robot_pos = robots[robot_idx][0]  # position (x, y)
            for j, pkg in enumerate(selected_packages):
                pkg_start = pkg.start  # start position (x, y)
                pkg_target = pkg.target
                # Khoảng cách Manhattan
                cost_matrix[i, j] = abs(robot_pos[0] - pkg_start[0]) + abs(robot_pos[1] - pkg_start[1]) + \
                                    abs(pkg_start[0] - pkg_target[0]) + abs(pkg_start[1] - pkg_start[1])

        # Chạy Hungarian Algorithm
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Tạo danh sách gán
        assignments = []
        for i, j in zip(row_ind, col_ind):
            robot_idx = available_robots[i]
            package_id = selected_packages[j][4]  # package id
            assignments.append((robot_idx, package_id))
            self.assignments[robot_idx] = package_id  # Lưu gán

        return assignments



    def update_inner_state(self, state):
        # Update robot positions and states
        for i in range(len(state['robots'])):
            # trước
            prev = (self.robots[i][0], self.robots[i][1], self.robots[i][2])
            # sau
            robot = state['robots'][i]
            self.robots[i] = (robot[0]-1, robot[1]-1, robot[2])
            print(i, self.robots[i])
            if prev[2] != 0:
                if self.robots[i][2] == 0:
                    # Robot has dropped the package
                    self.robots_target[i] = 'free'
                else:
                    self.robots_target[i] = self.robots[i][2]
        
        # Update package positions and states
        # self.packages += [(p[0], p[1]-1, p[2]-1, p[3]-1, p[4]-1, p[5]) for p in state['packages']]
        # self.packages_free += [True] * len(state['packages'])    

    def get_actions(self, state):
        if self.is_init == False:
            # This mean we have invoke the init agents, use the update_inner_state to update the state
            self.is_init = True
            self.update_inner_state(state)

        else:
            self.update_inner_state(state)

        actions = []
        print("State robot: ", self.robots)
        # Start assigning a greedy strategy
        for i in range(self.n_robots):
            # Step 1: Check if the robot is already assigned to a package
            if self.robots_target[i] != 'free':
                
                closest_package_id = self.robots_target[i]
                # Step 1b: Check if the robot has reached the package
                if self.robots[i][2] != 0:
                    # Move to the target points
                    
                    move, action = self.update_move_to_target(i, closest_package_id-1, 'target')
                    actions.append((move, str(action)))
                else:  
                    # Step 1c: Continue to move to the package
                    move, action = self.update_move_to_target(i, closest_package_id-1)    
                    actions.append((move, str(action)))
            else:
                # Step 2: Find a package to pick up
                # Find the closest package
                closest_package_id = None
                closed_distance = 1000000
                for j in np.arange(len(self.packages)):
                    # nếu package đã được assign
                    if not self.packages_free[j]:
                        continue
                    
                    # manhattan distance
                    pkg = self.packages[j]                
                    d = abs(pkg[1]-self.robots[i][0]) + abs(pkg[2]-self.robots[i][1])
                    # d += abs(pkg[1]-pkg[3]) + abs(pkg[2]-pkg[4])
                    if d < closed_distance:
                        closed_distance = d
                        closest_package_id = pkg[0]
                    print('d =', d, 'closest =', closed_distance, 'package id = ', closest_package_id)

                if closest_package_id is not None:
                    self.packages_free[closest_package_id-1] = False
                    self.robots_target[i] = closest_package_id
                    move, action = self.update_move_to_target(i, closest_package_id-1)    
                    actions.append((move, str(action)))
                else:
                    actions.append(('S', '0'))

                print(self.packages_free)

        print("N robots = ", len(self.robots))
        print("Actions = ", actions)
        print(self.robots_target)
        return actions
