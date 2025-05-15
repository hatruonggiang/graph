import pygame
import sys

class GridMapViewer:
    def __init__(self, cell_size=30):
        pygame.init()
        self.cell_size = cell_size
        self.font = pygame.font.SysFont(None, 24)

        self.button_rect = pygame.Rect(0, 0, 10, 10)
        self.clock = pygame.time.Clock()

    def draw(self, grid, robots, packages, step):
        rows = len(grid)
        cols = len(grid[0])
        width = cols * self.cell_size
        height = rows * self.cell_size + 60  # space for button/info

        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Grid Map Viewer")

        # Copy grid to draw
        grid_copy = [row[:] for row in grid]
        for i, robot in enumerate(robots):
            r, c = robot.position
            grid_copy[r][c] = f'R{i+1}'

        self.screen.fill((255, 255, 255))

        for i in range(rows):
            for j in range(cols):
                rect = pygame.Rect(j * self.cell_size, i * self.cell_size, self.cell_size, self.cell_size)
                val = grid_copy[i][j]

                if val == 1:
                    pygame.draw.rect(self.screen, (0, 0, 0), rect)
                else:
                    pygame.draw.rect(self.screen, (255, 255, 255), rect)
                    pygame.draw.rect(self.screen, (0, 0, 0), rect, 1)

                    if isinstance(val, str):
                        text = self.font.render(val, True, (0, 0, 255))
                        text_rect = text.get_rect(center=rect.center)
                        self.screen.blit(text, text_rect)
        
        # Draw packages
        # Vẽ các gói hàng
        # Vẽ các gói hàng
        for i, package in enumerate(packages):
            start_r, start_c = package.start
            target_r, target_c = package.target

            # Chọn màu theo trạng thái
            if package.status == 'None':
                color = (255, 0, 0)         # Đỏ
            elif package.status == 'Picked':
                color = (255, 165, 0)       # Cam
            elif package.status == 'Delivered':
                color = (128, 128, 128)     # Xám
            else:
                color = (100, 100, 255)     # Mặc định nếu status lạ

            # Vẽ tại vị trí bắt đầu
            start_rect = pygame.Rect(start_c * self.cell_size, start_r * self.cell_size, self.cell_size, self.cell_size)
            pygame.draw.rect(self.screen, color, start_rect)
            pygame.draw.rect(self.screen, (0, 0, 0), start_rect, 1)
            start_text = self.font.render(f'P{i+1}', True, (0, 0, 0))
            start_text_rect = start_text.get_rect(center=start_rect.center)
            self.screen.blit(start_text, start_text_rect)

            # Vẽ đích đến
            target_rect = pygame.Rect(target_c * self.cell_size, target_r * self.cell_size, self.cell_size, self.cell_size)
            pygame.draw.rect(self.screen, (0, 255, 0), target_rect)
            pygame.draw.rect(self.screen, (0, 0, 0), target_rect, 1)
            target_text = self.font.render(f'G{i+1}', True, (0, 0, 0))
            target_text_rect = target_text.get_rect(center=target_rect.center)
            self.screen.blit(target_text, target_text_rect)


        # Vẽ các robot sau khi vẽ gói hàng
        for i, robot in enumerate(robots):
            robot_r, robot_c = robot.position  # Vị trí robot

            # Nếu robot và gói hàng ở cùng một ô
            for package in packages:
                # Vị trí bắt đầu của gói hàng
                package_start_r, package_start_c = package.start
                package_end_r, package_end_c = package.target

                if (robot_r == package_start_r and robot_c == package_start_c) or (robot_r == package_end_r and robot_c == package_end_c):
                    # Vẽ robot lên gói hàng nếu ở cùng ô
                    robot_rect = pygame.Rect(robot_c * self.cell_size, robot_r * self.cell_size, self.cell_size, self.cell_size)
                    pygame.draw.rect(self.screen, (0, 0, 255), robot_rect)  # Màu xanh dương cho robot
                    pygame.draw.rect(self.screen, (0, 0, 0), robot_rect, 1)  # Vẽ viền cho robot
                    robot_text = self.font.render(f'R{i+1}', True, (0, 0, 0))  # Nhãn robot
                    robot_text_rect = robot_text.get_rect(center=robot_rect.center)
                    self.screen.blit(robot_text, robot_text_rect)
        # Draw step
        step_text = self.font.render(f"Step: {step}", True, (0, 0, 0))
        self.screen.blit(step_text, (10, height - 50))

        # Draw button
        self.button_rect = pygame.Rect(width - 120, height - 50, 100, 10)
        pygame.draw.rect(self.screen, (0, 200, 0), self.button_rect)
        text = self.font.render("Next", True, (255, 255, 255))
        text_rect = text.get_rect(center=self.button_rect.center)
        self.screen.blit(text, text_rect)

        pygame.display.flip()

    def wait_for_click(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if self.button_rect.collidepoint(event.pos):
                        return
            self.clock.tick(10)


# Ví dụ sử dụng
class Package:
    def __init__(self, start):
        self.start = start

if __name__ == "__main__":
    grid_map = [
        [0, 1, 0],
        [0, 0, 1],
        [0, 0, 0]
    ]
    
    robots = [
        type("Robot", (), {"position": (1, 0)}),  # Robot 1 at position (1, 0)
        type("Robot", (), {"position": (0, 2)}),  # Robot 2 at position (0, 2)
    ]

    packages = [
        Package(start=(2, 1)),  # Package at (2, 1)
        Package(start=(0, 1))   # Package at (0, 1)
    ]

    viewer = GridMapViewer()
    viewer.draw(grid_map, robots, packages, 1)
    viewer.wait_for_click()
