import pygame
import sys

class GridMapViewer:
    def __init__(self, grid_maps, cell_size=50):
        pygame.init()
        self.grid_maps = grid_maps
        self.index = 0
        self.cell_size = cell_size

        self.rows = len(grid_maps[0])
        self.cols = len(grid_maps[0][0])
        self.width = self.cols * cell_size
        self.height = self.rows * cell_size

        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Grid Map Viewer")
        self.font = pygame.font.SysFont(None, 24)

    def draw_map(self):
        self.screen.fill((255, 255, 255))  # clear
        grid = self.grid_maps[self.index]

        for i in range(self.rows):
            for j in range(self.cols):
                val = grid[i][j]
                rect = pygame.Rect(j * self.cell_size, i * self.cell_size, self.cell_size, self.cell_size)

                if val == 1:
                    color = (0, 0, 0)
                    pygame.draw.rect(self.screen, color, rect)
                else:
                    color = (255, 255, 255)
                    pygame.draw.rect(self.screen, color, rect)
                    # pygame.draw.rect(self.screen, (0, 0, 0), rect, 1)  # border

                    # if isinstance(val, str):
                    #     if val.startswith('s'):
                    #         text = self.font.render(, True, (0, 0, 255))
                    #         text_rect = text.get_rect(center=rect.center)
                    #         self.screen.blit(text, text_rect)

                    if isinstance(val, str):
                        if val.startswith('S'):
                            pygame.draw.rect(self.screen, (94, 235, 225), rect)
                        elif val.startswith('T'):
                            pygame.draw.rect(self.screen, (94, 235, 120), rect)
                        text = self.font.render(val, True, (0, 0, 255))
                        text_rect = text.get_rect(center=rect.center)
                        self.screen.blit(text, text_rect)

                    # pygame.draw.rect(self.screen, color, rect)
                    pygame.draw.rect(self.screen, (0, 0, 0), rect, 1)  # border

        map_text = self.font.render(f"Map {self.index + 1} / {len(self.grid_maps)}", True, (255, 0, 0))
        self.screen.blit(map_text, (10, 10))

        pygame.display.flip()

    def run(self):
        clock = pygame.time.Clock()

        while True:
            self.draw_map()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RIGHT:
                        if self.index < len(self.grid_maps) - 1:
                            self.index += 1
                    elif event.key == pygame.K_LEFT:
                        if self.index > 0:
                            self.index -= 1

            clock.tick(30)


# Ví dụ sử dụng
if __name__ == "__main__":
    grid_maps = [
        [
            [0, 1, 0],
            ['R1', 0, 1],
            [1, 0, 'R2']
        ],
        [
            [1, 1, 1],
            [0, 'R3', 0],
            [0, 0, 0]
        ]
    ]

    viewer = GridMapViewer(grid_maps)
    viewer.run()