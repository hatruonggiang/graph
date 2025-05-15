def step(self, actions):
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
                         r -= 0.1
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
                             r += 0.1
             else:
                 pkg_id = robot.carrying
                 target = self.packages[pkg_id - 1].target
                 dist = abs(target[0] - robot.position[0]) + abs(target[1] - robot.position[1])
                 if move in ['L', 'R', 'U', 'D'] and final_positions[i] != robot.position:
                     new_dist = abs(target[0] - final_positions[i][0]) + abs(target[1] - final_positions[i][1])
                     if new_dist < dist:
                         r += 0.1
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
         infos['no_package_steps'] = sum(1 for robot in self.robots if robot.carrying == 0)  # Tính mỗi bước
         infos['total_distance'] = sum(1 for i, (move, _) in enumerate(actions) if move in ['L', 'R', 'U', 'D'] and final_positions[i] != self.robots[i].position)  # Tính mỗi bước
         if infos['no_package_steps'] > 0:
             r -= 0.05 * infos['no_package_steps'] / len(self.robots)
         if self.check_terminate():
             done = True
             infos['total_reward'] = self.total_reward
             infos['total_time_steps'] = self.t
         return self.get_state(), r, done, infos
