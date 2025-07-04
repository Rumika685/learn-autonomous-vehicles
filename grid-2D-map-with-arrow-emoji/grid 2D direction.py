class DirectionalCar:
    def __init__(self, width=10, height=5):
        self.x = 0
        self.y = 2
        self.map_width = width
        self.map_height = height
        self.grid = [["⬜" for _ in range(width)] for _ in range(height)]
        self.route = []
        self.directions = ['east', 'south', 'west', 'north']
        self.current_dir_index = 0  # 最初は東向き
        self.arrow_map = {'east':  '➡️','south': '⬇️','west':  '⬅️','north': '⬆️'}

    def move(self, move_list):
        for move in move_list:
            if move == 'left':
                self.current_dir_index = (self.current_dir_index - 1) % 4
            elif move == 'right':
                self.current_dir_index = (self.current_dir_index + 1) % 4
            elif move == 'forward':
                direction = self.directions[self.current_dir_index]
                dx, dy = {
                    'east':  (1, 0),
                    'south': (0, 1),
                    'west':  (-1, 0),
                    'north': (0, -1)
                }[direction]

                # 移動
                self.route.append((self.x, self.y))
                self.grid[self.y][self.x] = self.arrow_map[direction]
                self.x += dx
                self.y += dy

                # マップ外に出ないように制限
                self.x = max(0, min(self.x, self.map_width - 1))
                self.y = max(0, min(self.y, self.map_height - 1))

        # スタートとゴールの印
        if self.route:
            sx, sy = self.route[0]
            gx, gy = self.x, self.y
            self.grid[sy][sx] = "🚗"
            self.grid[gy][gx] = "🏁"

    def print_map(self):
        for row in self.grid:
            print("".join(f" {cell} " for cell in row))


# 🔧 実行例（左・右・前の動き）
moves = ['forward', 'forward', 'right', 'forward', 'left', 'forward', 'forward']

car = DirectionalCar()
car.move(moves)
car.print_map()
