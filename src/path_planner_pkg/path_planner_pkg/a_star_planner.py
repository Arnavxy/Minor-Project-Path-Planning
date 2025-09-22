import heapq

class Grid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = set()

    def add_obstacle(self, obstacle):
        self.obstacles.add(obstacle)

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, x, y):
        return (x, y) not in self.obstacles

    def get_neighbors(self, node):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                x, y = node.x + dx, node.y + dy
                if self.in_bounds(x, y) and self.passable(x, y):
                    neighbors.append((x, y))
        return neighbors

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash((self.x, self.y))

def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)

def a_star_search(grid, start, end):
    start_node = Node(start[0], start[1])
    end_node = Node(end[0], end[1])

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node == end_node:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(current_node)

        for next_x, next_y in grid.get_neighbors(current_node):
            neighbor = Node(next_x, next_y, current_node)
            if neighbor in closed_set:
                continue

            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor, end_node)
            neighbor.f = neighbor.g + neighbor.h

            # This check is inefficient. A better way is to allow duplicates in the
            # open list and let the heap property handle picking the best one.
            # The `if neighbor in closed_set:` check prevents cycles.
            heapq.heappush(open_list, neighbor)

    return None