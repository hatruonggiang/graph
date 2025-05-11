import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.widgets import Button

class RobotVisualizer:
    def __init__(self, env, agents):
        self.env = env
        self.agents = agents
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.map = np.array(env.grid)  # Giả sử env.grid là bản đồ
        self.robot_positions = np.array([robot[:2] for robot in self.agents.robots])  # Vị trí của các robot
        self.t = 0  # Khởi tạo bước thời gian
        self.done = False  # Mặc định chưa hoàn thành

        # Tạo đồ họa bản đồ
        self.ax.set_xticks(np.arange(self.map.shape[1]))
        self.ax.set_yticks(np.arange(self.map.shape[0]))
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        # Vẽ các vật cản và các ô trống trong môi trường
        for r in range(self.map.shape[0]):
            for c in range(self.map.shape[1]):
                if self.map[r][c] == 1:  # Vật cản
                    self.ax.add_patch(plt.Rectangle((c, r), 1, 1, color='gray'))
                else:  # Ô trống
                    self.ax.add_patch(plt.Rectangle((c, r), 1, 1, color='white'))

        # Vẽ các robot và gói hàng
        self.robot_scatter = self.ax.scatter([], [], color='red', label='Robot')
        self.package_scatter = self.ax.scatter([], [], color='blue', label='Package')
        self.ax.legend()

        # Thêm nút để tăng bước thời gian
        self.ax_button = plt.axes([0.7, 0.01, 0.2, 0.075])  # Vị trí của nút
        self.button = Button(self.ax_button, 'Next Step')
        self.button.on_clicked(self.next_step)  # Khi nhấn nút, gọi hàm next_step

    def next_step(self, event):
        if not self.done:
            actions = self.agents.get_actions(self.env.state)
            next_state, reward, self.done, infos = self.env.step(actions)
            self.env.state = next_state
            self.t += 1  # Tăng bước thời gian
            self.update(self.t)  # Cập nhật giao diện với mốc thời gian mới
            self.ax.set_title(f'Time Step: {self.t}')  # Cập nhật tiêu đề với bước thời gian
            self.fig.canvas.draw()  # Vẽ lại đồ thị

    def update(self, frame):
        # Cập nhật vị trí của các robot
        self.robot_positions = np.array([robot[:2] for robot in self.agents.robots])  # Vị trí mới của robots
        self.robot_scatter.set_offsets(self.robot_positions)

        # Lấy vị trí của các gói hàng (giả sử bạn có thông tin gói hàng)
        package_positions = np.array([pkg[1:3] for pkg in self.agents.packages])  # Vị trí gói hàng
        self.package_scatter.set_offsets(package_positions)

        # Làm mới các ô vật cản và ô trống
        for r in range(self.map.shape[0]):
            for c in range(self.map.shape[1]):
                if self.map[r][c] == 1:  # Vật cản
                    self.ax.add_patch(plt.Rectangle((c, r), 1, 1, color='gray'))
                else:  # Ô trống
                    self.ax.add_patch(plt.Rectangle((c, r), 1, 1, color='white'))

        return self.robot_scatter, self.package_scatter

    def run(self):
        # Hiển thị bản đồ và nút
        plt.show()
