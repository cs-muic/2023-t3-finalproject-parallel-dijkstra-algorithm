use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: usize,
    position: usize,
}

// The priority queue depends on `Ord`.
impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // We reverse the order to make the priority queue a min-heap.
        other.cost.cmp(&self.cost)
    }
}

// `PartialOrd` needs to be implemented as well.
impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn sequential_dijkstra(graph: &Vec<Vec<(usize, usize)>>, start: usize) -> Vec<usize> {
    // Initialize distances with `usize::MAX` indicating infinity.
    let mut dist: Vec<_> = (0..graph.len()).map(|_| usize::MAX).collect();
    // Initialize the priority queue (min-heap).
    let mut heap = BinaryHeap::new();

    // Start with the initial node, distance 0.
    dist[start] = 0;
    heap.push(State { cost: 0, position: start });

    while let Some(State { cost, position }) = heap.pop() {
        // If the cost is greater than the recorded shortest distance, skip it.
        if cost > dist[position] {
            continue;
        }

        // For each neighbor of the current node...
        for &(next, weight) in &graph[position] {
            let next_cost = cost + weight;

            // If a shorter path to the neighbor is found...
            if next_cost < dist[next] {
                // Update the shortest path to the neighbor.
                dist[next] = next_cost;
                // Push the updated state to the heap.
                heap.push(State { cost: next_cost, position: next });
            }
        }
    }

    dist
}

#[cfg(test)]
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_graph() {
        let graph = vec![
            vec![(1, 2), (2, 4)],  // Node 0 is connected to Node 1 (cost 2) and Node 2 (cost 4)
            vec![(2, 1)],          // Node 1 is connected to Node 2 (cost 1)
            vec![]                 // Node 2 has no outgoing edges
        ];
        let result = sequential_dijkstra(&graph, 0);
        assert_eq!(result, vec![0, 2, 3]);  // Shortest paths: 0->0 (cost 0), 0->1 (cost 2), 0->2 (cost 3)
    }

    #[test]
    fn test_disconnected_graph() {
        let graph = vec![
            vec![(1, 2)],  // Node 0 is connected to Node 1 (cost 2)
            vec![],        // Node 1 has no outgoing edges
            vec![]         // Node 2 is disconnected
        ];
        let result = sequential_dijkstra(&graph, 0);
        assert_eq!(result, vec![0, 2, usize::MAX]);  // Node 2 is unreachable from Node 0
    }

    #[test]
    fn test_larger_graph() {
        let graph = vec![
            vec![(1, 1), (2, 4), (3, 7)],  // Node 0 connections
            vec![(3, 2)],                  // Node 1 connections
            vec![(3, 1)],                  // Node 2 connections
            vec![]                         // Node 3 has no outgoing edges
        ];
        let result = sequential_dijkstra(&graph, 0);
        assert_eq!(result, vec![0, 1, 4, 3]);  // Shortest paths: 0->0 (cost 0), 0->1 (cost 1), 0->2 (cost 4), 0->3 (cost 3)
    }

    #[test]
    fn test_complex_graph() {
        let graph = vec![
            vec![(1, 10), (2, 3)],                 // Node 0
            vec![(2, 1), (3, 2)],                  // Node 1
            vec![(1, 4), (3, 8), (4, 2)],          // Node 2
            vec![(4, 7)],                          // Node 3
            vec![(3, 9)],                          // Node 4
        ];
        let result = sequential_dijkstra(&graph, 0);
        assert_eq!(result, vec![0, 7, 3, 9, 5]);  // Shortest paths: 0->0 (cost 0), 0->1 (cost 7), 0->2 (cost 3), 0->3 (cost 9), 0->4 (cost 5)
    }

    #[test]
    fn test_very_complex_graph() {
        let graph = vec![
            vec![(1, 5), (2, 1), (3, 10)],     // Node 0
            vec![(0, 5), (2, 3), (4, 1)],      // Node 1
            vec![(0, 1), (1, 3), (3, 4), (4, 8)],  // Node 2
            vec![(0, 10), (2, 4), (4, 2)],     // Node 3
            vec![(1, 1), (2, 8), (3, 2), (5, 6)],  // Node 4
            vec![(4, 6)],                      // Node 5
        ];
        let result = sequential_dijkstra(&graph, 0);
        assert_eq!(result, vec![0, 4, 1, 5, 5, 11]);  // Shortest paths: 0->0 (cost 0), 0->1 (cost 4), 0->2 (cost 1), 0->3 (cost 5), 0->4 (cost 5), 0->5 (cost 11)
    }
}
