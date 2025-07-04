class Simple2DCar:
    def __init__(self, width=10, height=5):
        self.x = 0  # スタート位置（左上）
        self.y = 2
        self.map_width = width
        self.map_height = height
        self.grid = [["⬜" for _ in range(width)] for _ in range(height)]
        self.route = []

    def move_forward(self, steps):
        for _ in range(steps):
            self.route.append((self.x, self.y))
            self.grid[self.y][self.x] = "→"
            self.x += 1  # 東（右）へ進む
            if self.x >= self.map_width:
                break
        # スタートとゴール印
        if self.route:
            sx, sy = self.route[0]
            gx, gy = self.route[-1]
            self.grid[sy][sx] = "🚗"
            self.grid[gy][gx] = "🏁"

    def print_map(self):
        for row in self.grid:
            print("".join(row))

# 実行
car = Simple2DCar()
car.move_forward(6)
car.print_map()
