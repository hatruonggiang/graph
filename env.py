import numpy as np

class Robot:
    def __init__(self, position):
        self.position = position
        self.carrying = 0

class Package:
    def __init__(self, start, start_time, target, deadline, package_id):
        self.start = start
        self.start_time = start_time
        self.target = target
        self.deadline = deadline
        self.package_id = package_id
        self.status = 'None'  # Possible statuses: 'waiting', 'in_transit', 'delivered'

class Environment:
    def __init__(self, map_file, max_time_steps=100, n_robots=5, n_packages=20,
                 move_cost=-0.005, delivery_reward=10.0, delay_reward=3.0, seed=2025):
        """ Initializes the simulation environment. """
        self.map_file = map_file
        self.grid = self.load_map()
        self.n_rows = len(self.grid)
        self.n_cols = len(self.grid[0]) if self.grid else 0
        self.move_cost = move_cost  # Giảm chi phí di chuyển
        self.delivery_reward = delivery_reward
        self.delay_reward = delay_reward  # Tăng thưởng giao trễ
        self.t = 0
        self.robots = []
        self.packages = []
        self.total_reward = 0
        self.n_robots = n_robots
        self.max_time_steps = max_time_steps
        self.n_packages = n_packages
        self.rng = np.random.RandomState(seed)
        self.reset()
        self.done = False
        self.state = None

    def load_map(self):
        """ Reads the map file and returns a 2D grid with random noise. """
        grid = []
        with open(self.map_file, 'r') as f:
            for line in f:
                row = [int(x) for x in line.strip().split(' ')]
                grid.append(row)
        # Thêm nhiễu: thay đổi 10% ô ngẫu nhiên
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if self.rng.random() < 0.1:
                    grid[i][j] = 1 if grid[i][j] == 0 else 0
        return grid

    def is_free_cell(self, position):
        """ Checks if the cell at the given position is free. """
        r, c = position
        if r < 0 or r >= self.n_rows or c < 0 or c >= self.n_cols:
            return False
        return self.grid[r][c] == 0

    def add_robot(self, position):
        """ Adds a robot at the given position if the cell is free. """
        if self.is_free_cell(position):
            robot = Robot(position)
            self.robots.append(robot)
        else:
            raise ValueError("Invalid robot position: must be on a free cell.")

    def reset(self):
        """ Resets the environment to its initial state. """
        self.t = 0
        self.robots = []
        self.packages = []
        self.total_reward = 0
        self.done = False
        self.state = None
        tmp_grid = np.array(self.grid)
        for i in range(self.n_robots):
            position, tmp_grid = self.get_random_free_cell(tmp_grid)
            self.add_robot(position)
        N = self.n_rows
        list_packages = []
        for i in range(self.n_packages):
            start = self.get_random_free_cell_p()
            while True:
                target = self.get_random_free_cell_p()
                if start != target:
                    break
            to_deadline = 10 + self.rng.randint(N/2, 3*N)
            if i <= min(self.n_robots, 20):
                start_time = 0
            else:
                start_time = self.rng.randint(1, self.max_time_steps)
            list_packages.append((start_time, start, target, start_time + to_deadline))
        list_packages.sort(key=lambda x: x[0])
        for i in range(self.n_packages):
            start_time, start, target, deadline = list_packages[i]
            package_id = i + 1
            self.packages.append(Package(start, start_time, target, deadline, package_id))
        return self.get_state()

    def get_state(self):
        """ Returns the current state with all waiting or in-transit packages. """
        selected_packages = [p for p in self.packages if p.status in ['waiting', 'in_transit'] or p.start_time == self.t]
        for p in selected_packages:
            if p.start_time == self.t and p.status == 'None':
                p.status = 'waiting'
        state = {
            'time_step': self.t,
            'map': self.grid,
            'robots': [(robot.position[0] + 1, robot.position[1] + 1, robot.carrying) for robot in self.robots],
            'packages': [(p.package_id, p.start[0] + 1, p.start[1] + 1, p.target[0] + 1, p.target[1] + 1, p.start_time, p.deadline) for p in selected_packages]
        }
        return state

    def get_random_free_cell_p(self):
        """ Returns a random free cell in the grid. """
        free_cells = [(i, j) for i in range(self.n_rows) for j in range(self.n_cols) if self.grid[i][j] == 0]
        i = self.rng.randint(0, len(free_cells))
        return free_cells[i]

    def get_random_free_cell(self, new_grid):
        """ Returns a random free cell in the grid. """
        free_cells = [(i, j) for i in range(self.n_rows) for j in range(self.n_cols) if new_grid[i][j] == 0]
        i = self.rng.randint(0, len(free_cells))
        new_grid[free_cells[i][0]][free_cells[i][1]] = 1
        return free_cells[i], new_grid

    def step(self, actions):
        """ Advances the simulation by one timestep with shaping rewards. """
        r = 0
        if len(actions) != len(self.robots):
            raise ValueError("The number of actions must match the number of robots.")
        proposed_positions = []
        old_pos = {}
        next_pos = {}
        for i, robot in enumerate(self.robots):
            move, pkg_act = actions[i]
            new_pos = self.compute_new_position(robot.position, move)
            if not self.valid_position(new_pos):
                new_pos = robot.position
            proposed_positions.append(new_pos)
            old_pos[robot.position] = i
            next_pos[new_pos] = i
        moved_robots = [0 for _ in range(len(self.robots))]
        computed_moved = [0 for _ in range(len(self.robots))]
        final_positions = [None] * len(self.robots)
        occupied = {}
        while True:
            updated = False
            for i in range(len(self.robots)):
                if computed_moved[i] != 0:
                    continue
                pos = self.robots[i].position
                new_pos = proposed_positions[i]
                can_move = False
                if new_pos not in old_pos:
                    can_move = True
                else:
                    j = old_pos[new_pos]
                    if (j != i) and (computed_moved[j] == 0):
                        continue
                    can_move = True
                if can_move:
                    if new_pos not in occupied:
                        occupied[new_pos] = i
                        final_positions[i] = new_pos
                        computed_moved[i] = 1
                        moved_robots[i] = 1
                        updated = True
                    else:
                        new_pos = pos
                        occupied[new_pos] = i
                        final_positions[i] = pos
                        computed_moved[i] = 1
                        moved_robots[i] = 0
                        r -= 0.1  # Phạt khi robot kẹt do va chạm
                        updated = True
                if updated:
                    break
            if not updated:
                break
        for i in range(len(self.robots)):
            if computed_moved[i] == 0:
                final_positions[i] = self.robots[i].position
        for i, robot in enumerate(self.robots):
            move, pkg_act = actions[i]
            if robot.carrying == 0:
                waiting_pkgs = [p for p in self.packages if p.status == 'waiting' and p.start_time <= self.t]
                if waiting_pkgs:
                    nearest_pkg = min(waiting_pkgs, key=lambda p: abs(p.start[0] - robot.position[0]) + abs(p.start[1] - robot.position[1]))
                    dist = abs(nearest_pkg.start[0] - robot.position[0]) + abs(nearest_pkg.start[1] - robot.position[1])
                    if move in ['L', 'R', 'U', 'D'] and final_positions[i] != robot.position:
                        new_dist = abs(nearest_pkg.start[0] - final_positions[i][0]) + abs(nearest_pkg.start[1] - final_positions[i][1])
                        if new_dist < dist:
                            r += 0.1  # Thưởng tiến gần gói hàng
            else:
                pkg_id = robot.carrying
                target = self.packages[pkg_id - 1].target
                dist = abs(target[0] - robot.position[0]) + abs(target[1] - robot.position[1])
                if move in ['L', 'R', 'U', 'D'] and final_positions[i] != robot.position:
                    new_dist = abs(target[0] - final_positions[i][0]) + abs(target[1] - final_positions[i][1])
                    if new_dist < dist:
                        r += 0.1  # Thưởng tiến gần mục tiêu
            if move in ['L', 'R', 'U', 'D'] and final_positions[i] != robot.position:
                r += self.move_cost
            robot.position = final_positions[i]
        for i, robot in enumerate(self.robots):
            move, pkg_act = actions[i]
            if pkg_act == '1':
                if robot.carrying == 0:
                    for j in range(len(self.packages)):
                        if self.packages[j].status == 'waiting' and self.packages[j].start == robot.position and self.packages[j].start_time <= self.t:
                            package_id = self.packages[j].package_id
                            robot.carrying = package_id
                            self.packages[j].status = 'in_transit'
                            break
            elif pkg_act == '2':
                if robot.carrying != 0:
                    package_id = robot.carrying
                    target = self.packages[package_id - 1].target
                    if robot.position == target:
                        pkg = self.packages[package_id - 1]
                        pkg.status = 'delivered'
                        if self.t <= pkg.deadline:
                            r += self.delivery_reward
                        else:
                            r += self.delay_reward
                        robot.carrying = 0
        self.t += 1
        self.total_reward += r
        done = False
        infos = {}
        infos['no_package_steps'] = sum(1 for robot in self.robots if robot.carrying == 0)
        infos['total_distance'] = sum(1 for i, (move, _) in enumerate(actions) if move in ['L', 'R', 'U', 'D'] and final_positions[i] != self.robots[i].position)
        if infos['no_package_steps'] > 0:
            r -= 0.05 * infos['no_package_steps'] / len(self.robots)  # Phạt khi không mang hàng
        if self.check_terminate():
            done = True
            infos['total_reward'] = self.total_reward
            infos['total_time_steps'] = self.t
        return self.get_state(), r, done, infos

    def check_terminate(self):
        if self.t == self.max_time_steps:
            return True
        for p in self.packages:
            if p.status != 'delivered':
                return False
        return True

    def compute_new_position(self, position, move):
        """ Computes the intended new position for a robot. """
        r, c = position
        if move == 'S':
            return (r, c)
        elif move == 'L':
            return (r, c - 1)
        elif move == 'R':
            return (r, c + 1)
        elif move == 'U':
            return (r - 1, c)
        elif move == 'D':
            return (r + 1, c)
        else:
            return (r, c)

    def valid_position(self, pos):
        """ Checks if the new position is valid. """
        r, c = pos
        if r < 0 or r >= self.n_rows or c < 0 or c >= self.n_cols:
            return False
        if self.grid[r][c] == 1:
            return False
        return True

    def render(self):
        """ Text-based rendering of the map. """
        grid_copy = [row[:] for row in self.grid]
        for i, robot in enumerate(self.robots):
            r, c = robot.position
            grid_copy[r][c] = 'R%i' % i
        for row in grid_copy:
            print('\t'.join(str(cell) for cell in row))

if __name__ == "__main__":
    env = Environment('map.txt', 10, 2, 5)
    state = env.reset()
    print("Initial State:", state)
    print("Initial State:")
    env.render()
    from greedyagent import GreedyAgents as Agents
    agents = Agents()
    agents.init_agents(state)
    print("Agents initialized.")
    list_actions = ['S', 'L', 'R', 'U', 'D']
    n_robots = len(state['robots'])
    done = False
    t = 0
    while not done:
        actions = agents.get_actions(state)
        state, reward, done, infos = env.step(actions)
        print("\nState after step:")
        env.render()
        print(f"Reward: {reward}, Done: {done}, Infos: {infos}")
        print("Total Reward:", env.total_reward)
        print("Time step:", env.t)
        print("Packages:", state['packages'])
        print("Robots:", state['robots'])
        t += 1
        if t == 100:
            break
