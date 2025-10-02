# Importing modules and libraries
from maze_visual import maze, agent
import sys
import argparse
import time
from collections import deque
import heapq
import math

ROWS = 20 # Number of rows in the maze
COLS = 20 # Number of columns in the maze
m = maze(ROWS, COLS) # Initialize the maze

# Load the maze from the csv file. You may need to change this path depending on where you save the files.
m.LoadMaze(loadMaze=r"C:\Users\ZA Traders\Downloads\RollNumber_PA1\RollNumber_PA1\Search\maze_config.csv", theme="dark")


# ----------------------------------
def DFS(maze, start, goal):
    '''
    This function should implement the Depth First Search algorithm.
    The inputs to this function are:
        maze: The maze object
        start: The start position of the agent as a tuple (x,y)
        goal: The goal position of the agent as a tuple (x,y)
    The function should return:
        a list containing all the positions visited by the search algorithm
        a list containing the positions in the final path from the start to the goal
    '''

    visited_positions = []
    path_to_goal = []

     # Direction priority 
    DIR_PRIORITY = ['N', 'W', 'S', 'E']
    MOVE_DIRS = {
        'N': (-1, 0),
        'W': (0, -1),
        'S': (1, 0),
        'E': (0, 1)
    }

    stack = [(start, [start])]
    visited = set()
    visited_positions = []

    while stack:
        current_position, path_to_goal = stack.pop()

        if current_position in visited:
            continue

        visited.add(current_position)
        visited_positions.append(current_position)

        if current_position == goal:
            return visited_positions, path_to_goal

        # Reverse the direction priority['N', 'W', 'S', 'E'] because stack is LIFO
        for direction in reversed(DIR_PRIORITY):
            if maze.maze_map[current_position][direction] == 1:
                dx, dy = MOVE_DIRS[direction]  # dx and dy are the changes in the x and y coordinates respectively
                neighbor = (current_position[0] + dx, current_position[1] + dy)

                if neighbor not in visited:
                    stack.append((neighbor, path_to_goal + [neighbor]))

    return visited_positions, []  # No path found


    




def BFS(maze, start, goal):
    '''
    This function should implement the Breadth First Search algorithm.
    The inputs to this function are:
        maze: The maze object
        start: The start position of the agent as a tuple (x,y)
        goal: The goal position of the agent as a tuple (x,y)
    The function should return:
        a list containing all the positions visited by the search algorithm
        a list containing the positions in the final path from the start to the goal
    '''
    

    DIR_PRIORITY = ['N', 'W', 'S', 'E']
    MOVE_DIRS = {
        'N': (-1, 0),
        'W': (0, -1),
        'S': (1, 0),
        'E': (0, 1)
    }

    visited_positions = []
    path_to_goal = []
    queue = deque([(start, [start])])

    while queue:
        current_position, path_to_goal = queue.popleft()

        if current_position in visited_positions:
            continue

        visited_positions.append(current_position)

        if current_position == goal:
            return visited_positions, path_to_goal

        for direction in DIR_PRIORITY:
            if maze.maze_map[current_position][direction] == 1:
                dx, dy = MOVE_DIRS[direction]
                neighbor = (current_position[0] + dx, current_position[1] + dy)

                if neighbor not in visited_positions:
                    queue.append((neighbor, path_to_goal + [neighbor]))

    return visited_positions, []  # No path found





def heuristic(position, goal):
    '''
    This function should implement Euclidean Distance as the heuristic function used in A* algorithm.
    The inputs to this function are:
        position: The current position of the agent as a tuple (x,y)
        goal: The goal position of the agent as a tuple (x,y)
    The function should return:
        the heuristic value of the given position
    '''
    x1, y1 = position
    x2, y2 = goal
    h = math.sqrt((x2 - x1)**2 + (y2 - y1)**2) # Euclidean distance formula
    return h
   

def AStar(maze, start, goal):
    '''
    This function should implement the A* algorithm.
    The inputs to this function are:
        maze: The maze object
        start: The start position of the agent as a tuple (x,y)
        goal: The goal position of the agent as a tuple (x,y)
    The function should return:
        a list containing all the positions visited by the search algorithm
        a list containing the positions in the final path from the start to the goal
    '''

    DIR_PRIORITY = ['N', 'W', 'S', 'E']
    MOVE_DIRS = {
        'N': (-1, 0),
        'W': (0, -1),
        'S': (1, 0),
        'E': (0, 1)
    }

    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start, [start]))  # (f, g, position, path)
    # the above will push least f value to the top of the heap
    visited = {}
    visited_positions = [] #stores all visited nodes

    while open_set:
        f, g, current_position, path_to_goal = heapq.heappop(open_set)

        if current_position in visited and visited[current_position] <= g:
            continue

        visited[current_position] = g
        visited_positions.append(current_position)

        if current_position == goal:
            return visited_positions, path_to_goal

        for direction in DIR_PRIORITY:
            if maze.maze_map[current_position][direction] == 1:
                dx, dy = MOVE_DIRS[direction]
                neighbor = (current_position[0] + dx, current_position[1] + dy)

                new_g = g + 1  # because we assume cost of moving one step is 1
                new_f = new_g + heuristic(neighbor, goal)

                if neighbor not in visited or new_g < visited.get(neighbor, float('inf')):
                    heapq.heappush(open_set, (new_f, new_g, neighbor, path_to_goal + [neighbor]))

    return visited_positions, []  # No path found


# -------------------------------------
# This part of the code calls the search algorithms implemented above and displays the results on the maze
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bfs", help="Run BFS", action="store_true")
    parser.add_argument("-d", "--dfs", help="Run DFS", action="store_true")
    parser.add_argument("-a", "--astar", help="Run A* Search", action="store_true")

    args = parser.parse_args()

    start = (ROWS, COLS)
    goal = (1,1)

    explored, path_to_goal = [], []
    algorithm_name = ""
    start_time = 0

    if args.bfs:
        algorithm_name = "Breadth-First Search (BFS)"
        print(f"Running {algorithm_name}...")
        start_time = time.time()
        explored, path_to_goal = BFS(m, start, goal)
    elif args.dfs:
        algorithm_name = "Depth-First Search (DFS)"
        print(f"Running {algorithm_name}...")
        start_time = time.time()
        explored, path_to_goal = DFS(m, start, goal)
    elif args.astar:
        algorithm_name = "A* Search"
        print(f"Running {algorithm_name}...")
        start_time = time.time()
        explored, path_to_goal = AStar(m, start, goal)
    else:
        print("No search algorithm specified. See help below.")
        parser.print_help()
        sys.exit()
    
    # --- Statistics Calculation and Printing ---
    if start_time > 0:
        end_time = time.time()
        execution_time = end_time - start_time
        path_length = len(path_to_goal)
        nodes_explored = len(explored)

        print("\n--- Search Algorithm Statistics ---")
        print(f"Algorithm: {algorithm_name}")
        print(f"Execution Time: {execution_time:.4f} seconds")
        print(f"Path Length: {path_length} steps")
        print(f"Nodes Explored: {nodes_explored} nodes")
        print("-------------------------------------\n")
    # -------------------------------------------

    # If a path was found, start visualization
    if path_to_goal:
        print("Starting visualization...")
        a = agent(m, ROWS, COLS, filled=True)
        b = agent(m, ROWS, COLS, color="red")

        m.tracePath({a: explored}, delay=20)
        m.tracePath({b: path_to_goal}, delay=50)

        m.run()
    else:
        print("No path to the goal was found!")


if __name__ == "__main__":
    main()