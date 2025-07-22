import tkinter as tk
import numpy as np
import time
import math
import heapq

# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0
        self.parent_j = 0
        self.f = float('inf')
        self.g = float('inf')
        self.h = 0

# Read input from a file
def read_input(file_path):
    with open(file_path, 'r') as f:
        rows, cols = map(int, f.readline().strip().split())
        num_obstacles = int(f.readline().strip())
        grid = [[1 for _ in range(cols)] for _ in range(rows)]

        for _ in range(num_obstacles):
            obstacle = list(map(int, f.readline().strip().split()))
            grid[obstacle[0]][obstacle[1]] = 0

        src = list(map(int, f.readline().strip().split()))
        dest = list(map(int, f.readline().strip().split()))

    return rows, cols, grid, src, dest

# Check cell validity
def is_valid(row, col, ROW, COL):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

def is_unblocked(grid, row, col):
    return grid[row][col] == 1

def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Heuristic calculations
def calculate_h_value(row, col, dest, method):
    if method == 'manhattan':
        return abs(row - dest[0]) + abs(col - dest[1])
    elif method == 'diagonal':
        dx = abs(row - dest[0])
        dy = abs(col - dest[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
    return 0

def trace_path(cell_details, dest):
    path = []
    row, col = dest
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row, col = temp_row, temp_col
    path.append((row, col))
    path.reverse()
    return path

def a_star_search(grid, src, dest, heuristic):
    ROW, COL = len(grid), len(grid[0])
    if not is_valid(src[0], src[1], ROW, COL) or not is_valid(dest[0], dest[1], ROW, COL):
        print("Source or destination is invalid")
        return None

    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or destination is blocked")
        return None

    if is_destination(src[0], src[1], dest):
        print("Already at destination")
        return [src]

    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    i, j = src[0], src[1]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    open_list = []
    heapq.heappush(open_list, (0.0, i, j))

    while open_list:
        f, i, j = heapq.heappop(open_list)
        closed_list[i][j] = True

        for dir in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_i, new_j = i + dir[0], j + dir[1]

            if is_valid(new_i, new_j, ROW, COL) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                if is_destination(new_i, new_j, dest):
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    return trace_path(cell_details, dest)

                g_new = cell_details[i][j].g + math.sqrt(dir[0]**2 + dir[1]**2)
                h_new = calculate_h_value(new_i, new_j, dest, method=heuristic)
                f_new = g_new + h_new

                if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                    heapq.heappush(open_list, (f_new, new_i, new_j))
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j

    print("Destination not found")
    return None

class PathfindingApp:
    def __init__(self, master, grid, path_manhattan, path_diagonal, src, dest):
        self.master = master
        self.grid = grid
        self.path_manhattan = path_manhattan
        self.path_diagonal = path_diagonal
        self.src = src
        self.dest = dest
        self.cell_size = 40

        self.create_layout()
        self.create_tables()
        self.animate_paths()

    def create_layout(self):
        self.frame_manhattan = tk.Frame(self.master, bg='white')
        self.frame_manhattan.grid(row=0, column=0)

        self.frame_diagonal = tk.Frame(self.master, bg='white')
        self.frame_diagonal.grid(row=0, column=1)

        self.canvas_manhattan = tk.Canvas(self.frame_manhattan, width=self.cell_size * len(self.grid[0]),
                                          height=self.cell_size * len(self.grid), bg='indigo')
        self.canvas_manhattan.pack()

        self.canvas_diagonal = tk.Canvas(self.frame_diagonal, width=self.cell_size * len(self.grid[0]),
                                         height=self.cell_size * len(self.grid), bg='indigo')
        self.canvas_diagonal.pack()

        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                color = 'white' if self.grid[i][j] == 1 else 'yellow'
                self.canvas_manhattan.create_rectangle(j * self.cell_size, i * self.cell_size,
                                                       (j + 1) * self.cell_size, (i + 1) * self.cell_size, fill=color)
                self.canvas_diagonal.create_rectangle(j * self.cell_size, i * self.cell_size,
                                                      (j + 1) * self.cell_size, (i + 1) * self.cell_size, fill=color)

        # Draw start and destination on both canvases
        for canvas in [self.canvas_manhattan, self.canvas_diagonal]:
            canvas.create_oval(self.src[1] * self.cell_size + 10, self.src[0] * self.cell_size + 10,
                               self.src[1] * self.cell_size + 30, self.src[0] * self.cell_size + 30, fill='red')
            canvas.create_oval(self.dest[1] * self.cell_size + 10, self.dest[0] * self.cell_size + 10,
                               self.dest[1] * self.cell_size + 30, self.dest[0] * self.cell_size + 30, fill='blue')

    def create_tables(self):
        self.table_frame = tk.Frame(self.master)
        self.table_frame.grid(row=1, column=0, columnspan=2)

        # Labels to store execution time, path, and total cost for each heuristic
        self.execution_time_labels = []
        self.path_labels = []
        self.total_cost_labels = []

        # Iterate over each heuristic (Manhattan and Diagonal)
        for i, heuristic in enumerate(["Manhattan", "Diagonal"]):
            frame = tk.Frame(self.table_frame, padx=20, pady=10, relief="solid", borderwidth=1)  # Add border to frame
            frame.grid(row=0, column=i, padx=10)  # Space between tables
            tk.Label(frame, text=f"{heuristic} Path", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=4)

            # Add headers with padding to separate columns
            headers = ["Step", "g(n)", "h(n)", "f(n)"]
            for col, header in enumerate(headers):
                tk.Label(frame, text=header, borderwidth=1, relief="solid", padx=5, pady=5).grid(row=1, column=col, sticky="nsew")

            # Create cells for each row in the path, adding padding and borders for grid lines
            num_steps = max(len(self.path_manhattan), len(self.path_diagonal))
            for j in range(num_steps):
                for k in range(4):
                    cell = tk.Label(frame, text="", borderwidth=1, relief="solid", padx=5, pady=5)
                    cell.grid(row=j + 2, column=k, sticky="nsew")

                # Ensure all cells expand equally
                for col in range(4):
                    frame.grid_columnconfigure(col, weight=1)

            # Add labels for execution time, path, and total cost
            exec_time_label = tk.Label(frame, text="Execution Time: ", font=("Arial", 9))
            exec_time_label.grid(row=num_steps + 3, column=0, columnspan=4, sticky="w", padx=5)
            self.execution_time_labels.append(exec_time_label)

            path_label = tk.Label(frame, text="Path: ", font=("Arial", 9))
            path_label.grid(row=num_steps + 4, column=0, columnspan=4, sticky="w", padx=5)
            self.path_labels.append(path_label)

            cost_label = tk.Label(frame, text="Total Cost: ", font=("Arial", 9))
            cost_label.grid(row=num_steps + 5, column=0, columnspan=4, sticky="w", padx=5)
            self.total_cost_labels.append(cost_label)

    def update_summary(self, index, execution_time, path, total_cost):
        """Update execution time, path, and total cost at the bottom of each table."""
        self.execution_time_labels[index].config(text=f"Execution Time: {execution_time:.2f} seconds")
        self.path_labels[index].config(text=f"Path: {path}")
        self.total_cost_labels[index].config(text=f"Total Cost: {total_cost:.2f}")


    def animate_paths(self):
        self.animate_path(self.canvas_manhattan, self.path_manhattan, "Manhattan", 0)
        self.animate_path(self.canvas_diagonal, self.path_diagonal, "Diagonal", 1)

    def animate_path(self, canvas, path, heuristic, table_column):
        g_n = 0  # Start with g(n) as 0 for the starting cell

        for index, (i, j) in enumerate(path):
            # Highlight the path step on the canvas
            canvas.create_rectangle(j * self.cell_size, i * self.cell_size,
                                    (j + 1) * self.cell_size, (i + 1) * self.cell_size, fill='cyan')

            # For each step, calculate Euclidean distance from previous cell to current cell
            if index > 0:
                prev_i, prev_j = path[index - 1]
                g_n += math.sqrt((i - prev_i) ** 2 + (j - prev_j) ** 2)  # Euclidean distance

            # Calculate h(n) for the current cell using the specified heuristic
            h_n = calculate_h_value(i, j, self.dest, method=heuristic.lower())
            f_n = g_n + h_n

            # Update the table with step, g(n), h(n), f(n)
            for k, val in enumerate([index + 1, round(g_n, 2), round(h_n, 2), round(f_n, 2)]):
                tk.Label(self.table_frame.winfo_children()[table_column], text=str(val)).grid(row=index + 2, column=k)

            self.master.update()
            time.sleep(0.3)

if __name__ == "__main__":
    file_path = 'input.txt'
    rows, cols, grid, src, dest = read_input(file_path)

    # Run A* for Manhattan heuristic and measure execution time
    start_time = time.time()
    path_manhattan = a_star_search(grid, src, dest, 'manhattan')
    execution_time_manhattan = time.time() - start_time

    # Calculate total cost for Manhattan path using Euclidean distance
    total_cost_manhattan = sum(
        math.sqrt((path_manhattan[i][0] - path_manhattan[i - 1][0]) ** 2 +
                  (path_manhattan[i][1] - path_manhattan[i - 1][1]) ** 2)
        for i in range(1, len(path_manhattan))
    ) if path_manhattan else 0

    # Run A* for Diagonal heuristic and measure execution time
    start_time = time.time()
    path_diagonal = a_star_search(grid, src, dest, 'diagonal')
    execution_time_diagonal = time.time() - start_time

    # Calculate total cost for Diagonal path using Euclidean distance
    total_cost_diagonal = sum(
        math.sqrt((path_diagonal[i][0] - path_diagonal[i - 1][0]) ** 2 +
                  (path_diagonal[i][1] - path_diagonal[i - 1][1]) ** 2)
        for i in range(1, len(path_diagonal))
    ) if path_diagonal else 0

    # Initialize the Tkinter app and update tables with the summary data
    root = tk.Tk()
    app = PathfindingApp(root, grid, path_manhattan, path_diagonal, src, dest)

    # Update the summary information at the bottom of each table
    app.update_summary(0, execution_time_manhattan, path_manhattan, total_cost_manhattan)
    app.update_summary(1, execution_time_diagonal, path_diagonal, total_cost_diagonal)

    root.mainloop()

