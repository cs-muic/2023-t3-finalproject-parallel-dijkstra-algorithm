use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::sync::{Arc, Mutex};

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: usize,
    position: usize,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost).then_with(|| self.position.cmp(&other.position))
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn reverse_adj_list(adj_list: &[Vec<(usize, usize)>]) -> Vec<Vec<(usize, usize)>> {
    let mut rev_adj_list = vec![Vec::new(); adj_list.len()];
    for (node, edges) in adj_list.iter().enumerate() {
        for &(neighbor, weight) in edges {
            rev_adj_list[neighbor].push((node, weight));
        }
    }
    rev_adj_list
}

pub fn parallel_bi_dijkstra(graph: Arc<Vec<Vec<(usize, usize)>>>, start: usize, goal: usize) -> (usize, Vec<usize>) {
    if start == goal {
        return (0, vec![start]);
    }

    let rev_graph = Arc::new(reverse_adj_list(&graph));
    let dist_fwd = Arc::new(Mutex::new(vec![usize::MAX; graph.len()]));
    let dist_bwd = Arc::new(Mutex::new(vec![usize::MAX; graph.len()]));
    let best_cost = Arc::new(Mutex::new(usize::MAX));
    let best_path = Arc::new(Mutex::new(Vec::new()));

    crossbeam::scope(|s| {
        let bc_clone = best_cost.clone();
        let bp_clone = best_path.clone();
        s.spawn(move |_| {
            search(graph, start, dist_fwd, bc_clone, bp_clone, true);
        });

        let bc_clone = best_cost.clone();
        let bp_clone = best_path.clone();
        s.spawn(move |_| {
            search(rev_graph, goal, dist_bwd, bc_clone, bp_clone, false);
        });
    }).unwrap();

    let cost = best_cost.lock().unwrap();
    let path = best_path.lock().unwrap().clone();
    (*cost, path)
}

fn search(graph: Arc<Vec<Vec<(usize, usize)>>>, start: usize, distances: Arc<Mutex<Vec<usize>>>, best_cost: Arc<Mutex<usize>>, best_path: Arc<Mutex<Vec<usize>>>, is_forward: bool) {
    let mut heap = BinaryHeap::new();
    heap.push(State { cost: 0, position: start });

    {
        let mut distances = distances.lock().unwrap();
        distances[start] = 0;
    }

    while let Some(State { cost, position }) = heap.pop() {
        if let Some(edges) = graph.get(position) {
            for &(neighbor, weight) in edges {
                let next_cost = cost + weight;
                let mut dist = distances.lock().unwrap();
                if next_cost < dist[neighbor] {
                    dist[neighbor] = next_cost;
                    drop(dist);  // Explicitly drop to unlock before possible locking in best_cost
                    heap.push(State { cost: next_cost, position: neighbor });
                    let mut bc = best_cost.lock().unwrap();
                    if next_cost < *bc {
                        *bc = next_cost;
                        let mut path = best_path.lock().unwrap();
                        if is_forward {
                            path.push(position);  // Append path in forward direction
                        } else {
                            path.insert(0, position);  // Prepend path in backward direction
                        }
                    }
                }
            }
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;
    use std::sync::Arc;

    #[test]
    fn test_simple_graph() {
        let graph = Arc::new(vec![
            vec![(1, 2), (2, 4)],  // Node 0 is connected to Node 1 (cost 2) and Node 2 (cost 4)
            vec![(2, 1)],          // Node 1 is connected to Node 2 (cost 1)
            vec![]                 // Node 2 has no outgoing edges
        ]);
        let start_time = Instant::now();
        let (cost, path) = parallel_bi_dijkstra(graph, 0, 2);
        let duration = start_time.elapsed();
        println!("Test Simple Graph - Time elapsed: {:?}", duration);
        assert_eq!(cost, 3);  // Shortest path cost: 3
        assert_eq!(path, vec![0, 1, 2]);  // Shortest path: 0 -> 1 -> 2
    }

    #[test]
    fn test_disconnected_graph() {
        let graph = Arc::new(vec![
            vec![(1, 2)],  // Node 0 is connected to Node 1 (cost 2)
            vec![],        // Node 1 has no outgoing edges
            vec![]         // Node 2 is disconnected
        ]);
        let start_time = Instant::now();
        let (cost, path) = parallel_bi_dijkstra(graph, 0, 2);
        let duration = start_time.elapsed();
        println!("Test Disconnected Graph - Time elapsed: {:?}", duration);
        assert_eq!(cost, usize::MAX);  // Node 2 is unreachable from Node 0
        assert_eq!(path, vec![]);  // No path exists
    }

    #[test]
    fn test_larger_graph() {
        let graph = Arc::new(vec![
            vec![(1, 1), (2, 4), (3, 7)],  // Node 0 connections
            vec![(3, 1)],                  // Node 1 connections
            vec![(3, 1)],                  // Node 2 connections
            vec![]                         // Node 3 has no outgoing edges
        ]);
        let start_time = Instant::now();
        let (cost, path) = parallel_bi_dijkstra(graph, 0, 3);
        let duration = start_time.elapsed();
        println!("Test Larger Graph - Time elapsed: {:?}", duration);
        assert_eq!(cost, 2);  // Shortest path cost: 2
        assert_eq!(path, vec![0, 1, 3]);  // Shortest path: 0 -> 1 -> 3
    }

    #[test]
    fn test_complex_graph() {
        let graph = Arc::new(vec![
            vec![(1, 10), (2, 3)],                 // Node 0
            vec![(2, 1), (3, 2)],                  // Node 1
            vec![(1, 4), (3, 8), (4, 2)],          // Node 2
            vec![(4, 7)],                          // Node 3
            vec![(3, 9)],                          // Node 4
        ]);
        let start_time = Instant::now();
        let (cost, path) = parallel_bi_dijkstra(graph, 0, 4);
        let duration = start_time.elapsed();
        println!("Test Complex Graph - Time elapsed: {:?}", duration);
        assert_eq!(cost, 5);  // Shortest path cost: 5
        assert_eq!(path, vec![0, 2, 4]);  // Shortest path: 0 -> 2 -> 4
    }

    #[test]
    fn test_very_complex_graph() {
        let graph = Arc::new(vec![
            vec![(1, 5), (2, 1), (3, 10)],     // Node 0
            vec![(0, 5), (2, 3), (4, 1)],      // Node 1
            vec![(0, 1), (1, 3), (3, 4), (4, 8)],  // Node 2
            vec![(0, 10), (2, 4), (4, 2)],     // Node 3
            vec![(1, 1), (2, 8), (3, 2), (5, 6)],  // Node 4
            vec![(4, 6)],                      // Node 5
        ]);
        let start_time = Instant::now();
        let (cost, path) = parallel_bi_dijkstra(graph, 0, 5);
        let duration = start_time.elapsed();
        println!("Test Very Complex Graph - Time elapsed: {:?}", duration);
        assert_eq!(cost, 11);  // Shortest path cost: 11
        assert_eq!(path, vec![0, 2, 1, 4, 5]);  // Shortest path: 0 -> 2 -> 1 -> 4 -> 5
    }


}