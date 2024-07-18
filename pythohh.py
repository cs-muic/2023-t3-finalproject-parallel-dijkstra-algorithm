import heapq
import random
import time
from collections import defaultdict
from itertools import count

class Graph:
    def __init__(self):
        self.edges = defaultdict(list)

    def add_edge(self, u, v, weight):
        self.edges[u].append((v, weight))
        self.edges[v].append((u, weight))

def bidirectional_dijkstra(graph, source, target):
    if source not in graph.edges or target not in graph.edges:
        raise ValueError(f"Either source {source} or target {target} is not in the graph")

    if source == target:
        return (0, [source])

    def weight(x, y):
        for v, w in graph.edges[x]:
            if v == y:
                return w
        return float('inf')

    push = heapq.heappush
    pop = heapq.heappop
    dists = [{}, {}]  # dictionary of final distances
    paths = [{source: [source]}, {target: [target]}]  # dictionary of paths
    fringe = [[], []]  # heap of (distance, node) for choosing node to expand
    seen = [{source: 0}, {target: 0}]  # dict of distances to seen nodes
    c = count()

    push(fringe[0], (0, next(c), source))
    push(fringe[1], (0, next(c), target))

    neighs = [graph.edges, graph.edges]
    finaldist = float('inf')
    finalpath = []
    dir = 1

    while fringe[0] and fringe[1]:
        # Alternate search direction
        dir = 1 - dir
        dist, _, v = pop(fringe[dir])
        if v in dists[dir]:
            continue
        dists[dir][v] = dist

        if v in dists[1 - dir]:
            totaldist = dists[0][v] + dists[1][v]
            if totaldist < finaldist:
                finaldist = totaldist
                revpath = list(reversed(paths[1][v]))
                finalpath = paths[0][v] + revpath[1:]

        for w, d in neighs[dir][v]:
            vwLength = dists[dir][v] + d
            if w in dists[dir]:
                if vwLength < dists[dir][w]:
                    raise ValueError("Contradictory paths found: negative weights?")
            elif w not in seen[dir] or vwLength < seen[dir][w]:
                seen[dir][w] = vwLength
                push(fringe[dir], (vwLength, next(c), w))
                paths[dir][w] = paths[dir][v] + [w]
                if w in seen[0] and w in seen[1]:
                    totaldist = seen[0][w] + seen[1][w]
                    if finalpath == [] or finaldist > totaldist:
                        finaldist = totaldist
                        revpath = paths[1][w][:]
                        revpath.reverse()
                        finalpath = paths[0][w] + revpath[1:]

    if finalpath:
        return (finaldist, finalpath)
    else:
        return (float('inf'), [])

def dijkstra(graph, source, target=None):
    dist = {source: 0}
    prev = {source: None}
    heap = [(0, source)]
    visited = set()

    while heap:
        (d, u) = heapq.heappop(heap)
        if u in visited:
            continue
        visited.add(u)
        if u == target:
            break
        for v, weight in graph.edges.get(u, []):
            alt = d + weight
            if v not in dist or alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(heap, (alt, v))
    
    if target is None:
        return dist, prev
    
    path = []
    u = target
    if u in prev or u == source:
        while u is not None:
            path.insert(0, u)
            u = prev[u]
    return dist.get(target, float('inf')), path

def generate_large_graph(num_nodes, edges_per_node, max_weight):
    graph = Graph()
    for node in range(num_nodes):
        for _ in range(edges_per_node):
            target = random.randint(0, num_nodes - 1)
            weight = random.randint(1, max_weight)
            if target != node:
                graph.add_edge(node, target, weight)
    return graph

def main():
    graph = generate_large_graph(10000, 10, 10)  # Adjust parameters for larger graphs
    
    start_node = 0
    end_node = 9999

    print("\nTest Case: Very Large Graph")

    start_time = time.time()
    cost, path = bidirectional_dijkstra(graph, start_node, end_node)
    duration = time.time() - start_time
    print(f"Bidirectional Dijkstra: cost = {cost}, path length = {len(path)}, duration = {duration:.6f} seconds")

    start_time = time.time()
    cost, path = dijkstra(graph, start_node, end_node)
    duration = time.time() - start_time
    print(f"Sequential Dijkstra: cost = {cost}, path length = {len(path)}, duration = {duration:.6f} seconds")

if __name__ == "__main__":
    main()
