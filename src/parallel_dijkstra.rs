use std::cmp::Ordering;
use std::collections::BinaryHeap;
use rayon::prelude::*;

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: usize,
    position: usize,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn parallel_dijkstra(graph: &Vec<Vec<(usize, usize)>>, start: usize, goal: Option<usize>) -> (usize, Vec<usize>) {
    let mut dist: Vec<_> = (0..graph.len()).map(|_| usize::MAX).collect();
    let mut prev: Vec<_> = (0..graph.len()).map(|_| None).collect();
    let mut heap = BinaryHeap::new();

    dist[start] = 0;
    heap.push(State { cost: 0, position: start });

    while let Some(State { cost, position }) = heap.pop() {
        if cost > dist[position] {
            continue;
        }

        // If we have reached the goal, exit early.
        if let Some(goal) = goal {
            if position == goal {
                return (dist[goal], reconstruct_path(&prev, start, goal));
            }
        }

        //  collect updates in parallel.
        let updates: Vec<_> = graph[position].par_iter().filter_map(|&(next, weight)| {
            let next_cost = cost + weight;
            if next_cost < dist[next] {
                Some((next, next_cost, position))
            } else {
                None
            }
        }).collect();

        // Apply the updates **sequentially**.
        for (next, next_cost, pos) in updates {
            dist[next] = next_cost;
            prev[next] = Some(pos);
            heap.push(State { cost: next_cost, position: next });
        }
    }

    if let Some(goal) = goal {
        (dist[goal], reconstruct_path(&prev, start, goal))
    } else {
        (usize::MAX, vec![])
    }
}

fn reconstruct_path(prev: &Vec<Option<usize>>, start: usize, goal: usize) -> Vec<usize> {
    let mut path = vec![];
    let mut current = goal;
    while let Some(p) = prev[current] {
        path.push(current);
        current = p;
        if current == start {
            path.push(start);
            break;
        }
    }
    path.reverse();
    path
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn test_simple_graph() {
        let graph = vec![
            vec![(1, 2), (2, 4)],  // Node 0 is connected to Node 1 (cost 2) and Node 2 (cost 4)
            vec![(2, 1)],          // Node 1 is connected to Node 2 (cost 1)
            vec![]                 // Node 2 has no outgoing edges
        ];
        let start_time = Instant::now();
        let (cost, path) = parallel_dijkstra(&graph, 0, Some(2));
        let duration = start_time.elapsed();
        println!("Test Simple Graph - Time elapsed: {:?}", duration);
        assert_eq!(cost, 3);  // Shortest path cost: 3
        assert_eq!(path, vec![0, 1, 2]);  // Shortest path: 0 -> 1 -> 2
    }

    #[test]
    fn test_disconnected_graph() {
        let graph = vec![
            vec![(1, 2)],  // Node 0 is connected to Node 1 (cost 2)
            vec![],        // Node 1 has no outgoing edges
            vec![]         // Node 2 is disconnected
        ];
        let start_time = Instant::now();

        let (cost, path) = parallel_dijkstra(&graph, 0, Some(2));
        let duration = start_time.elapsed();
        println!("Test Disconnected Graph - Time elapsed: {:?}", duration);

        assert_eq!(cost, usize::MAX);  // Node 2 is unreachable from Node 0
        assert_eq!(path, vec![]);  // No path exists
    }

    #[test]
    fn test_larger_graph() {
        let graph = vec![
            vec![(1, 1), (2, 4), (3, 7)],  // Node 0 connections
            vec![(3, 1)],                  // Node 1 connections
            vec![(3, 1)],                  // Node 2 connections
            vec![]                         // Node 3 has no outgoing edges
        ];
        let start_time = Instant::now();

        let (cost, path) = parallel_dijkstra(&graph, 0, Some(3));
        let duration = start_time.elapsed();
        println!("Test Larger Graph - Time elapsed: {:?}", duration);

        assert_eq!(cost, 2);  // Shortest path cost: 2
        assert_eq!(path, vec![0, 1, 3]);  // Shortest path: 0 -> 1 -> 3
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
        let start_time = Instant::now();

        let (cost, path) = parallel_dijkstra(&graph, 0, Some(4));
        let duration = start_time.elapsed();
        println!("Test Complex Graph - Time elapsed: {:?}", duration);

        assert_eq!(cost, 5);  // Shortest path cost: 5
        assert_eq!(path, vec![0, 2, 4]);  // Shortest path: 0 -> 2 -> 4
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
        let start_time = Instant::now();

        let (cost, path) = parallel_dijkstra(&graph, 0, Some(5));
        let duration = start_time.elapsed();
        println!("Test Very Complex Graph - Time elapsed: {:?}", duration);

        assert_eq!(cost, 11);  // Shortest path cost: 11
        assert_eq!(path, vec![0, 2, 1, 4, 5]);  // Shortest path: 0 -> 2 -> 1 -> 4 -> 5
    }



}

/*
 successes:

 ---- parallel_dijkstra::tests::test_simple_graph stdout ----
Test Simple Graph - Time elapsed: 678.583µs

---- parallel_dijkstra::tests::test_larger_graph stdout ----
Test Larger Graph - Time elapsed: 674.708µs

---- parallel_dijkstra::tests::test_disconnected_graph stdout ----
Test Disconnected Graph - Time elapsed: 632.208µs

---- parallel_dijkstra::tests::test_complex_graph stdout ----
Test Complex Graph - Time elapsed: 696.791µs

---- parallel_dijkstra::tests::test_very_complex_graph stdout ----
Test Very Complex Graph - Time elapsed: 762.833µs
 */
