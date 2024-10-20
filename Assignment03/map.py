import tkinter as tk
from Assignment03.maze_data import dijkstra_multi_target, convert_path_to_dict, get_maze_data

def create_matrix_map(maze_size, traveled_path, start, targets, chicks):
    maze_map = [['X' for _ in range(maze_size[1])] for _ in range(maze_size[0])]
    for cell in traveled_path:
        maze_map[cell[0]][cell[1]] = '.'  # Mark the traveled path with a dot
    
    maze_map[start[0]][start[1]] = '-S-'  # Mark the start position

    for target in targets:
        if maze_map[target[0]][target[1]] == 'C':
            maze_map[target[0]][target[1]] = 'TC'  # Mark as target-chick
        elif maze_map[target[0]][target[1]] == '.':
            maze_map[target[0]][target[1]] = 'T'  # Mark as target-chick
        
        else:
            maze_map[target[0]][target[1]] = 'T'  # Mark as target

    for chick in chicks:
        if maze_map[chick[0]][chick[1]] == 'T':
            maze_map[chick[0]][chick[1]] = 'TC'  # Mark as target-chick
        elif maze_map[chick[0]][chick[1]] == '.':
            maze_map[chick[0]][chick[1]] = 'C.'  # Mark as target-chick
        else:
            maze_map[chick[0]][chick[1]] = 'C'  # Mark as chick

    return maze_map

def draw_maze(maze_map, paths, targets, chicks, target_distances):
    root = tk.Tk()
    root.title("Maze Map with Shortest Path")

    cell_size = 75
    canvas_width = len(maze_map[0]) * cell_size
    canvas_height = len(maze_map) * cell_size

    canvas = tk.Canvas(root, width=canvas_width, height=canvas_height)
    canvas.pack()

    target_count_label = tk.Label(root, text=f"Targets: {len(targets)}")
    target_count_label.pack()

    chick_count_label = tk.Label(root, text=f"Chicks: {len(chicks)}")
    chick_count_label.pack()

    distances_label = tk.Label(root, text="Target Distances:")
    distances_label.pack()

    distances_text = tk.Text(root, height=5, width=50)
    for target, distance in target_distances.items():
        distances_text.insert(tk.END, f"Target at {target}: Distance {distance}\n")
    distances_text.pack()

    # Draw the maze cells
    for i in range(len(maze_map)):
        for j in range(len(maze_map[i])):
            x0 = j * cell_size
            y0 = i * cell_size
            x1 = x0 + cell_size
            y1 = y0 + cell_size

            canvas.create_rectangle(x0, y0, x1, y1, fill="white", outline="black")

            text = maze_map[i][j]

            if text == '-S-':
                r = 70
                canvas.create_oval(
                    x0 + 30, y0 + 30, x0 + 25+r , y0 + 25+r  ,
                    fill="lightgreen", outline="black"
                )

            elif text == 'T':
                r = 30
                canvas.create_oval(
                    x0 + 5, y0 + 5, x0 + 5 + r, y0 + 5 + r,
                    fill="red", outline="red"
                )
            elif text == 'C':
                r = 30
                canvas.create_oval(
                    x1 - 5 - r, y1 - 5 - r, x1 - 5, y1 - 5,
                    fill="yellow", outline="yellow"
                )
            elif text == 'C.':
                r = 30
                canvas.create_oval(
                    x1 - 5 - r, y1 - 5 - r, x1 - 5, y1 - 5,
                    fill="yellow", outline="yellow"
                )
            elif text == 'TC':
                r = 30
                canvas.create_oval(
                    x0 + 5, y0 + 5, x0 + 5 + r, y0 + 5 + r,
                    fill="red", outline="red"
                )
                canvas.create_oval(
                    x1 - 5 - r, y1 - 5 - r, x1 - 5, y1 - 5,
                    fill="yellow", outline="yellow"
                )
            elif text == 'T.':
                r = 30
                canvas.create_oval(
                    x0 + 5, y0 + 5, x0 + 5 + r, y0 + 5 + r,
                    fill="red", outline="red"
                )
                canvas.create_oval(
                    x0 + 20 + r, y0 + 20 + r , x1 - 20-r, y1 - 20-r,
                    fill="grey"
                )

            # Draw an "X" for cells in the traveled path
            if (i, j) in traveled_path:
                r = 35
                canvas.create_oval(
                    x0 + 20 + r, y0 + 20 + r , x1 - 20-r, y1 - 20-r,
                    fill="darkgrey"
                )

    # Show full traveled path initially
    for i in range(len(traveled_path) - 1):
        x0 = traveled_path[i][1] * cell_size + cell_size // 2
        y0 = traveled_path[i][0] * cell_size + cell_size // 2
        x1 = traveled_path[i + 1][1] * cell_size + cell_size // 2
        y1 = traveled_path[i + 1][0] * cell_size + cell_size // 2
        canvas.create_line(x0, y0, x1, y1, fill="darkblue", width=3,tags="line")

    def show_shortest_path():
        canvas.delete("line")  # Remove previously drawn lines
        for path in paths.values():
            for i in range(len(path) - 1):
                x0 = path[i][1] * cell_size + cell_size // 2
                y0 = path[i][0] * cell_size + cell_size // 2
                x1 = path[i + 1][1] * cell_size + cell_size // 2
                y1 = path[i + 1][0] * cell_size + cell_size // 2
                canvas.create_line(x0, y0, x1, y1, fill="orange", width=5, tags="line")  # Use a different color for clarity

    shortest_path_button = tk.Button(root, text="Show Shortest Path", command=show_shortest_path)
    shortest_path_button.pack()

    root.mainloop()

if __name__ == "__main__":
    shortest_path_list = []
    # Get maze data from maze_logic.py
    maze_size, traveled_path, maze, start, targets, chicks = get_maze_data()

    # Calculate shortest paths and distances
    target_distances, paths = dijkstra_multi_target(maze, start, targets, chicks)

    # Create the maze map for display
    maze_map = create_matrix_map(maze_size, traveled_path, start, targets, chicks)

    # Draw the maze with the full traveled path initially
    draw_maze(maze_map, paths, targets, chicks, target_distances)
