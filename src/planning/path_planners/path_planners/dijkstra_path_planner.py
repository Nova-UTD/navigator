import heapq
import numpy as np


class DijkstraPathPlanner:
    def __init__(self):
        pass

    def shortest_path(self, costmap, start, end, obstacle_threshold=90):
        # Fast heapq Dijkstra — no NetworkX graph construction.
        # Operates directly on the numpy costmap array.
        if costmap[start] >= obstacle_threshold or costmap[end] >= obstacle_threshold:
            return None

        rows, cols = costmap.shape
        dist = np.full((rows, cols), np.inf, dtype=np.float64)
        dist[start] = 0.0
        prev = {}
        heap = [(0.0, start)]
        DIRS = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
        SQRT2 = 1.4142135623730951

        while heap:
            d, (r, c) = heapq.heappop(heap)
            if (r, c) == end:
                break
            if d > dist[r, c]:
                continue
            for dr, dc in DIRS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                cell_val = int(costmap[nr, nc])
                if cell_val >= obstacle_threshold:
                    continue
                move_cost = float(cell_val + 1) * (SQRT2 if dr and dc else 1.0)
                nd = d + move_cost
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    prev[(nr, nc)] = (r, c)
                    heapq.heappush(heap, (nd, (nr, nc)))

        if end not in prev and start != end:
            return None

        path = []
        cur = end
        while cur in prev:
            path.append(cur)
            cur = prev[cur]
        path.append(start)
        path.reverse()
        return path
