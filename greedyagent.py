# import numpy as np
# Run a BFS to find the path from start to goal
def run_bfs(map, start, goal):
    # Hàm tìm đường đi ngắn nhất từ điểm bắt đầu đến điểm đích sử dụng thuật toán BFS
    # map: Bản đồ 2D (0 là ô trống, khác 0 là chướng ngại vật)
    # start: Tọa độ điểm bắt đầu (hàng, cột)
    # goal: Tọa độ điểm đích (hàng, cột)
    
    n_rows = len(map)                # Số hàng của bản đồ
    n_cols = len(map[0])             # Số cột của bản đồ

    queue = []                       # Hàng đợi cho BFS
    visited = set()                  # Tập hợp các ô đã thăm
    queue.append((goal, []))         # Bắt đầu BFS từ điểm đích (ngược lại với BFS thông thường)
    visited.add(goal)                # Đánh dấu điểm đích đã thăm
    d = {}                           # Từ điển lưu khoảng cách từ điểm đích đến mỗi ô
    d[goal] = 0                      # Khoảng cách từ điểm đích đến chính nó là 0

    # Thực hiện BFS
    while queue:
        current, path = queue.pop(0)     # Lấy ô hiện tại và đường đi đến ô đó
        
        # Kiểm tra 4 hướng di chuyển có thể (lên, xuống, trái, phải)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            next_pos = (current[0] + dx, current[1] + dy)    # Tính tọa độ ô kế tiếp
            
            # Kiểm tra xem ô kế tiếp có nằm trong bản đồ không
            if next_pos[0] < 0 or next_pos[0] >= n_rows or next_pos[1] < 0 or next_pos[1] >= n_cols:
                continue    # Bỏ qua nếu ô nằm ngoài bản đồ
                
            # Kiểm tra xem ô kế tiếp chưa được thăm và là ô trống (có thể đi qua)
            if next_pos not in visited and map[next_pos[0]][next_pos[1]] == 0:
                visited.add(next_pos)                    # Đánh dấu ô đã thăm
                d[next_pos] = d[current] + 1             # Cập nhật khoảng cách
                queue.append((next_pos, path + [next_pos]))    # Thêm ô vào hàng đợi với đường đi mới

    # Kiểm tra xem có thể đến được điểm bắt đầu không
    if start not in d:
        return 'S', 100000    # Nếu không thể đến được, trả về 'S' (đứng yên) và khoảng cách rất lớn
    
    # Xác định bước đi đầu tiên từ điểm bắt đầu
    t = 0
    actions = ['U', 'D', 'L', 'R']    # Các hành động có thể: lên, xuống, trái, phải
    current = start
    
    # Kiểm tra 4 hướng di chuyển từ điểm bắt đầu
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        next_pos = (current[0] + dx, current[1] + dy)    # Tính tọa độ ô kế tiếp
        
        # Nếu ô kế tiếp nằm trong đường đi
        if next_pos in d:
            # Nếu ô kế tiếp gần đích hơn (khoảng cách nhỏ hơn)
            if d[next_pos] == d[current] - 1:
                return actions[t], d[next_pos]    # Trả về hướng di chuyển và khoảng cách
        t += 1    # Tăng chỉ số hành động
        
    # Nếu không tìm thấy hướng di chuyển tốt hơn, đứng yên
    return 'S', d[start]





class GreedyAgents:
    # Lớp GreedyAgents triển khai thuật toán tham lam để điều khiển nhiều robot vận chuyển gói hàng

    def __init__(self):
        # Khởi tạo các biến cần thiết
        self.agents = []          # Danh sách các tác nhân (không được sử dụng trong mã hiện tại)
        self.packages = []        # Danh sách các gói hàng cần vận chuyển
        self.packages_free = []   # Danh sách boolean đánh dấu gói hàng nào chưa được phân công
        self.n_robots = 0         # Số lượng robot
        self.state = None         # Lưu trữ trạng thái hiện tại

        self.is_init = False      # Cờ đánh dấu đã khởi tạo hay chưa

    def init_agents(self, state):
        # Khởi tạo trạng thái ban đầu từ dữ liệu đầu vào
        self.state = state                        # Lưu trạng thái
        self.n_robots = len(state['robots'])      # Lấy số lượng robot
        self.map = state['map']                   # Lưu bản đồ môi trường
        # Chuyển đổi vị trí robot (trừ 1 vì có thể chỉ số bắt đầu từ 1)
        self.robots = [(robot[0]-1, robot[1]-1, 0) for robot in state['robots']]
        # Khởi tạo mục tiêu của robot là 'free' (chưa được gán gói hàng)
        self.robots_target = ['free'] * self.n_robots
        # Thêm các gói hàng vào danh sách với định dạng (id, vị_trí_x, vị_trí_y, đích_x, đích_y, ?)
        self.packages += [(p[0], p[1]-1, p[2]-1, p[3]-1, p[4]-1, p[5]) for p in state['packages']]
        # Đánh dấu tất cả gói hàng là chưa được phân công
        self.packages_free = [True] * len(self.packages)

    def update_move_to_target(self, robot_id, target_package_id, phase='start'):
        # Tính toán hướng di chuyển tiếp theo cho robot

        if phase == 'start':
            # Nếu đang trong giai đoạn 'start': Tính khoảng cách từ robot đến vị trí gói hàng
            distance = abs(self.packages[target_package_id][1]-self.robots[robot_id][0]) + \
            abs(self.packages[target_package_id][2]-self.robots[robot_id][1])
        else:
            # Nếu đang trong giai đoạn 'target': Tính khoảng cách từ robot đến điểm đích của gói hàng
            distance = abs(self.packages[target_package_id][3]-self.robots[robot_id][0]) + \
            abs(self.packages[target_package_id][4]-self.robots[robot_id][1])
        i = robot_id
        #print(self.robots[i], distance, phase)

        # Bước 4: Di chuyển đến gói hàng hoặc điểm đích
        pkg_act = 0               # Mặc định không có hành động với gói hàng
        move = 'S'                # Mặc định đứng yên
        if distance >= 1:
            # Nếu còn khoảng cách, tìm đường đi tối ưu
            pkg = self.packages[target_package_id]
            
            # Xác định điểm đích dựa vào giai đoạn
            target_p = (pkg[1], pkg[2])  # Vị trí gói hàng
            if phase == 'target':
                target_p = (pkg[3], pkg[4])  # Điểm đích của gói hàng
            
            # Sử dụng BFS để tìm đường đi tối ưu
            move, distance = run_bfs(self.map, (self.robots[i][0], self.robots[i][1]), target_p)

            if distance == 0:
                # Nếu đã đến đích, thực hiện hành động tương ứng
                if phase == 'start':
                    pkg_act = 1  # Nhặt gói hàng
                else:
                    pkg_act = 2  # Thả gói hàng
        else:
            # Nếu đã ở vị trí đích, thực hiện hành động tương ứng
            move = 'S'  # Đứng yên
            if phase == 'start':
                pkg_act = 1  # Nhặt gói hàng
            else:
                pkg_act = 2  # Thả gói hàng

        return move, str(pkg_act)
    
    def update_inner_state(self, state):
        # Cập nhật trạng thái nội bộ dựa trên trạng thái mới
        
        # Cập nhật vị trí và trạng thái của robot
        for i in range(len(state['robots'])):
            prev = (self.robots[i][0], self.robots[i][1], self.robots[i][2])  # Lưu trạng thái trước đó
            robot = state['robots'][i]
            # Cập nhật vị trí và trạng thái mới
            self.robots[i] = (robot[0]-1, robot[1]-1, robot[2])
            # print(i, self.robots[i])
            
            if prev[2] != 0:  # Nếu trước đó robot đang mang gói hàng
                if self.robots[i][2] == 0:
                    # Nếu hiện tại không mang gói hàng nữa, robot đã thả gói
                    self.robots_target[i] = 'free'  # Đánh dấu robot là tự do
                else:
                    # Nếu vẫn mang gói hàng, cập nhật ID gói hàng đang mang
                    self.robots_target[i] = self.robots[i][2]
        
         # Cập nhật gói hàng nếu chưa tồn tại
        existing_ids = {pkg[0] for pkg in self.packages}  # package_id là phần tử thứ 0
        for p in state['packages']:
            package_id = p[0] - 1
            if package_id not in existing_ids:
                self.packages.append((package_id, p[1]-1, p[2]-1, p[3]-1, p[4] - 1, p[5]))  # p[5] là status
                self.packages_free.append(True) 

    def get_actions(self, state):
        # Phương thức chính để quyết định hành động cho mỗi robot
        if self.is_init == False:
            # Nếu chưa khởi tạo, sử dụng update_inner_state để cập nhật trạng thái
            self.is_init = True
            self.update_inner_state(state)
        else:
            # Nếu đã khởi tạo, chỉ cập nhật trạng thái
            self.update_inner_state(state)

        actions = []  # Danh sách hành động cho tất cả robot
        print("State robot: ", self.robots)
        
        # Bắt đầu áp dụng chiến lược tham lam
        for i in range(self.n_robots):
            print(f"Robot {i} : {self.robots_target[i]})")
            # Bước 1: Kiểm tra xem robot đã được gán gói hàng chưa
            if self.robots_target[i] != 'free':
                # Robot đã được gán gói hàng
                closest_package_id = self.robots_target[i]
                
                # Bước 1b: Kiểm tra xem robot đã nhặt gói hàng chưa
                if self.robots[i][2] != 0:
                    # Robot đang mang gói hàng, di chuyển đến điểm đích
                    move, action = self.update_move_to_target(i, closest_package_id-1, 'target')
                    actions.append((move, str(action)))
                    print(f"Robot {i} đang mang gói hàng {closest_package_id}, mục tiêu hiện tại là {self.packages[closest_package_id-1][3]},{self.packages[closest_package_id-1][4]}")  # In target của robot
                else:  
                    # Robot chưa nhặt gói hàng, tiếp tục di chuyển đến vị trí gói
                    move, action = self.update_move_to_target(i, closest_package_id-1)    
                    actions.append((move, str(action)))
            else:
                # Bước 2: Tìm gói hàng để nhặt
                # Tìm gói hàng gần nhất chưa được phân công
                closest_package_id = None
                closed_distance = 1000000  # Khởi tạo với giá trị lớn
                
                for j in range(len(self.packages)):
                    if not self.packages_free[j]:
                        # Bỏ qua gói hàng đã được phân công
                        continue

                    pkg = self.packages[j]                
                    # Tính khoảng cách Manhattan từ robot đến gói hàng
                    d = abs(pkg[1]-self.robots[i][0]) + abs(pkg[2]-self.robots[i][1])
                    
                    if d < closed_distance:
                        # Nếu tìm thấy gói hàng gần hơn, cập nhật
                        closed_distance = d
                        closest_package_id = pkg[0]

                if closest_package_id is not None:
                    # Nếu tìm thấy gói hàng, phân công cho robot
                    self.packages_free[closest_package_id-1] = False  # Đánh dấu gói hàng đã được phân công
                    self.robots_target[i] = closest_package_id  # Gán gói hàng cho robot
                    # Tính toán hướng di chuyển đến gói hàng
                    move, action = self.update_move_to_target(i, closest_package_id-1)    
                    actions.append((move, str(action)))
                    print(f"Robot {i} đang tìm tới gói hàng {closest_package_id}, mục tiêu hiện tại là {self.packages[closest_package_id-1][1]},{self.packages[closest_package_id-1][2]}")  # In start của package mà robot hướng tới
                else:
                    # Nếu không tìm thấy gói hàng nào, robot đứng yên
                    actions.append(('S', '0'))

        # In thông tin debug
        print("N robots = ", len(self.robots))
        print("Actions = ", actions)
        print(self.robots_target)
        
        return actions  # Trả về danh sách hành động cho tất cả robot
    def resolve_conflicts_and_update(robot_positions, robot_moves, map, package_positions, robot_status, robot_goals):
        n_rows = len(map)
        n_cols = len(map[0])
        
        # Tính toán vị trí dự kiến sau khi di chuyển
        move_map = {'U': (-1, 0), 'D': (1, 0), 'L': (0, -1), 'R': (0, 1), 'S': (0, 0)}
        intended_positions = []
        
        for i, move in enumerate(robot_moves):
            dx, dy = move_map[move]
            cur_x, cur_y = robot_positions[i]
            new_x, new_y = cur_x + dx, cur_y + dy

            # Kiểm tra giới hạn bản đồ và chướng ngại vật
            if 0 <= new_x < n_rows and 0 <= new_y < n_cols and map[new_x][new_y] == 0:
                intended_positions.append((new_x, new_y))
            else:
                # Nếu di chuyển không hợp lệ => giữ nguyên
                intended_positions.append((cur_x, cur_y))
        
        # Phát hiện xung đột (nhiều robot cùng vào 1 ô)
        final_positions = robot_positions[:]
        pos_to_robot = {}
        
        for i, pos in enumerate(intended_positions):
            if pos not in pos_to_robot:
                pos_to_robot[pos] = [i]
            else:
                pos_to_robot[pos].append(i)
        
        for pos, robots in pos_to_robot.items():
            if len(robots) == 1:
                final_positions[robots[0]] = pos
            else:
                # Giữ robot có chỉ số nhỏ nhất, các robot khác giữ nguyên
                winner = min(robots)
                final_positions[winner] = pos
                for r in robots:
                    if r != winner:
                        final_positions[r] = robot_positions[r]
        
        # Xử lý nhặt hoặc thả gói hàng
        updated_package_positions = package_positions[:]
        carrying = robot_status[:]

        for i, pos in enumerate(final_positions):
            # TH1: đang mang hàng và đến đích => thả hàng
            if carrying[i] and pos == robot_goals[i]:
                carrying[i] = False

            # TH2: chưa mang hàng và đang đứng trên gói => nhặt
            elif not carrying[i]:
                # Kiểm tra xem tại đây có gói hàng nào không
                items_here = [j for j, p in enumerate(package_positions) if p == pos]
                if items_here:
                    picked = min(items_here)  # gói có chỉ số nhỏ nhất
                    carrying[i] = True
                    updated_package_positions[picked] = None  # đánh dấu đã nhặt
        
        return final_positions, updated_package_positions, carrying


