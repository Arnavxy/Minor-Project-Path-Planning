import heapq

class Grid:
    def __init__(self, width, height, depth):
        self.width = width
        self.height = height
        self.depth = depth
        self.obstacles = set()
 
    def add_obstacle(self, obstacle):
        self.obstacles.add(obstacle)
 
    def in_bounds(self, x, y, z):
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth
 
    def passable(self, x, y, z):
        return (x, y, z) not in self.obstacles
 
    def get_neighbors(self, node):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    
                    x, y, z = node.x + dx, node.y + dy, node.z + dz
                    if self.in_bounds(x, y, z) and self.passable(x, y, z):
                        neighbors.append((x, y, z))
        return neighbors
 
class Node:
    def __init__(self, x, y, z, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0
 
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z
 
    def __lt__(self, other):
        return self.f < other.f
 
    def __hash__(self):
        return hash((self.x, self.y, self.z))
 
def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z)

def distance_to_obstacle(node, obstacles):
    min_dist = float('inf')
    for obs in obstacles:
        dist = ((node.x - obs[0])**2 + (node.y - obs[1])**2 + (node.z - obs[2])**2)**0.5
        if dist < min_dist:
            min_dist = dist
    return min_dist

def a_star_search(grid, start, end):
    start_node = Node(start[0], start[1], start[2])
    end_node = Node(end[0], end[1], end[2])
  
    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node == end_node:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y, current_node.z))
                current_node = current_node.parent
            return path[::-1]
  
        closed_set.add(current_node)
  
        for next_x, next_y, next_z in grid.get_neighbors(current_node):
            neighbor = Node(next_x, next_y, next_z, current_node)
            if neighbor in closed_set:
                continue
  
            dist_to_obs = distance_to_obstacle(neighbor, grid.obstacles)
            obstacle_cost = 0
            if dist_to_obs < 2.0:  # Add a high cost if the node is too close to an obstacle
                obstacle_cost = 100 * (2.0 - dist_to_obs)

            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor, end_node)
            neighbor.f = neighbor.g + neighbor.h + obstacle_cost

            heapq.heappush(open_list, neighbor)

    return None