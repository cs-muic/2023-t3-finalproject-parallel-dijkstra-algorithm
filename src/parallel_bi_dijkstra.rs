use rayon::prelude::*;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::sync::{Arc, Mutex};
use rand::{distributions::{Distribution, Uniform}, SeedableRng, rngs::StdRng, Rng};


#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: usize,
    position: usize,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
            .then_with(|| self.position.cmp(&other.position))
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

pub fn parallel_bidirectional_dijkstra(graph: &[Vec<(usize, usize)>], start: usize, goal: usize) -> (usize, Vec<usize>) {
    if start == goal {
        return (0, vec![start]);
    }

    let rev_graph = reverse_adj_list(graph);
    let graph = Arc::new(graph.to_vec());
    let rev_graph = Arc::new(rev_graph);
    let dist_fwd = Arc::new(Mutex::new(vec![usize::MAX; graph.len()]));
    let dist_bwd = Arc::new(Mutex::new(vec![usize::MAX; graph.len()]));
    let heap_fwd = Arc::new(Mutex::new(BinaryHeap::new()));
    let heap_bwd = Arc::new(Mutex::new(BinaryHeap::new()));
    let prev_fwd = Arc::new(Mutex::new(vec![None; graph.len()]));
    let prev_bwd = Arc::new(Mutex::new(vec![None; graph.len()]));
    let estimate = Arc::new(Mutex::new(usize::MAX));
    let join_node = Arc::new(Mutex::new(None));

    {
        let mut dist_fwd = dist_fwd.lock().unwrap();
        let mut dist_bwd = dist_bwd.lock().unwrap();
        let mut heap_fwd = heap_fwd.lock().unwrap();
        let mut heap_bwd = heap_bwd.lock().unwrap();
        
        dist_fwd[start] = 0;
        dist_bwd[goal] = 0;
        heap_fwd.push(State { cost: 0, position: start });
        heap_bwd.push(State { cost: 0, position: goal });
    }

    rayon::scope(|s| {
        s.spawn(|_| {
            while let Some(State { cost, position }) = { heap_fwd.lock().unwrap().pop() } {
                let mut dist_fwd = dist_fwd.lock().unwrap();
                let mut prev_fwd = prev_fwd.lock().unwrap();
                if cost > dist_fwd[position] {
                    continue;
                }
                for &(neighbor, weight) in &graph[position] {
                    let next_cost = cost.saturating_add(weight);
                    if next_cost < dist_fwd[neighbor] {
                        dist_fwd[neighbor] = next_cost;
                        heap_fwd.lock().unwrap().push(State { cost: next_cost, position: neighbor });
                        prev_fwd[neighbor] = Some(position);
                    }
                }
                let dist_bwd = dist_bwd.lock().unwrap();
                if let Some(&rev_cost) = dist_bwd.get(position) {
                    if rev_cost != usize::MAX {
                        let total_cost = dist_fwd[position].saturating_add(rev_cost);
                        let mut estimate = estimate.lock().unwrap();
                        if total_cost < *estimate {
                            *estimate = total_cost;
                            *join_node.lock().unwrap() = Some(position);
                        }
                    }
                }
            }
        });

        s.spawn(|_| {
            while let Some(State { cost, position }) = { heap_bwd.lock().unwrap().pop() } {
                let mut dist_bwd = dist_bwd.lock().unwrap();
                let mut prev_bwd = prev_bwd.lock().unwrap();
                if cost > dist_bwd[position] {
                    continue;
                }
                for &(neighbor, weight) in &rev_graph[position] {
                    let next_cost = cost.saturating_add(weight);
                    if next_cost < dist_bwd[neighbor] {
                        dist_bwd[neighbor] = next_cost;
                        heap_bwd.lock().unwrap().push(State { cost: next_cost, position: neighbor });
                        prev_bwd[neighbor] = Some(position);
                    }
                }
                let dist_fwd = dist_fwd.lock().unwrap();
                if let Some(&fwd_cost) = dist_fwd.get(position) {
                    if fwd_cost != usize::MAX {
                        let total_cost = dist_bwd[position].saturating_add(fwd_cost);
                        let mut estimate = estimate.lock().unwrap();
                        if total_cost < *estimate {
                            *estimate = total_cost;
                            *join_node.lock().unwrap() = Some(position);
                        }
                    }
                }
            }
        });
    });

    let (final_cost, final_path) = {
        let join = *join_node.lock().unwrap();
        if let Some(join) = join {
            let mut path_fwd = reconstruct_path(join, &prev_fwd.lock().unwrap());
            let mut path_bwd = reconstruct_path(join, &prev_bwd.lock().unwrap());
            path_bwd.reverse();
            path_fwd.pop(); // Avoid duplicate join node
            path_fwd.extend(path_bwd);
            (*estimate.lock().unwrap(), path_fwd)
        } else {
            (usize::MAX, Vec::new())
        }
    };

    (final_cost, final_path)
}

fn reconstruct_path(goal: usize, prev: &[Option<usize>]) -> Vec<usize> {
    let mut path = Vec::new();
    let mut current = Some(goal);
    while let Some(node) = current {
        path.push(node);
        current = prev[node];
    }
    path.reverse();
    path
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    fn generate_random_graph(nodes: usize, edges: usize) -> Vec<Vec<(usize, usize)>> {
        let mut graph = vec![Vec::new(); nodes];
        let mut rng = rand::thread_rng();
        let range = rand::distributions::Uniform::from(0..nodes);
        let weight_range = rand::distributions::Uniform::from(1..100);

        for _ in 0..edges {
            let u = range.sample(&mut rng);
            let v = range.sample(&mut rng);
            if u != v {
                let weight = weight_range.sample(&mut rng);
                graph[u].push((v, weight));
            }
        }

        graph
    }

    #[test]
    fn test_large_simple_graph() {
        let graph = generate_random_graph(1000, 5000);
        let start_time = Instant::now();
        let (cost, path) = parallel_bidirectional_dijkstra(&graph, 0, 999);
        let duration = start_time.elapsed();
        println!("Par-BiDijkstra Test Large Simple Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }
    #[test]
    fn test_simple_graph() {
        let graph = vec![
            vec![(1, 2), (2, 4)],  // Node 0 is connected to Node 1 (cost 2) and Node 2 (cost 4)
            vec![(2, 1)],          // Node 1 is connected to Node 2 (cost 1)
            vec![]                 // Node 2 has no outgoing edges
        ];
        let start_time = Instant::now();
        let (cost, path) = parallel_bidirectional_dijkstra(&graph, 0, 2);
        let duration = start_time.elapsed();
        println!("Bi-Test Simple Graph - Time elapsed: {:?}", duration);
        assert_eq!(cost, 3);  // Shortest path cost: 3
        assert_eq!(path, vec![0, 1, 2]);  // Shortest path: 0 -> 1 -> 2
    }}



// #[cfg(test)]
// mod tests {
//     use super::*;
//     use std::time::Instant;
//     use std::sync::Arc;

//     #[test]
//     fn test_simple_graph() {
//         let graph = Arc::new(vec![
//             vec![(1, 2), (2, 4)],  // Node 0 is connected to Node 1 (cost 2) and Node 2 (cost 4)
//             vec![(2, 1)],          // Node 1 is connected to Node 2 (cost 1)
//             vec![]                 // Node 2 has no outgoing edges
//         ]);
//         let start_time = Instant::now();
//         let (cost, path) = parallel_bi_dijkstra(graph, 0, 2);
//         let duration = start_time.elapsed();
//         println!("Test Simple Graph - Time elapsed: {:?}", duration);
//         assert_eq!(cost, 3);  // Shortest path cost: 3
//         assert_eq!(path, vec![0, 1, 2]);  // Shortest path: 0 -> 1 -> 2
//     }
    

//     #[test]
//     fn test_disconnected_graph() {
//         let graph = Arc::new(vec![
//             vec![(1, 2)],  // Node 0 is connected to Node 1 (cost 2)
//             vec![],        // Node 1 has no outgoing edges
//             vec![]         // Node 2 is disconnected
//         ]);
//         let start_time = Instant::now();
//         let (cost, path) = parallel_bi_dijkstra(graph, 0, 2);
//         let duration = start_time.elapsed();
//         println!("Test Disconnected Graph - Time elapsed: {:?}", duration);
//         assert_eq!(cost, usize::MAX);  // Node 2 is unreachable from Node 0
//         assert_eq!(path, vec![]);  // No path exists
//     }

//     #[test]
//     fn test_larger_graph() {
//         let graph = Arc::new(vec![
//             vec![(1, 1), (2, 4), (3, 7)],  // Node 0 connections
//             vec![(3, 1)],                  // Node 1 connections
//             vec![(3, 1)],                  // Node 2 connections
//             vec![]                         // Node 3 has no outgoing edges
//         ]);
//         let start_time = Instant::now();
//         let (cost, path) = parallel_bi_dijkstra(graph, 0, 3);
//         let duration = start_time.elapsed();
//         println!("Test Larger Graph - Time elapsed: {:?}", duration);
//         assert_eq!(cost, 2);  // Shortest path cost: 2
//         assert_eq!(path, vec![0, 1, 3]);  // Shortest path: 0 -> 1 -> 3
//     }

//     #[test]
//     fn test_complex_graph() {
//         let graph = Arc::new(vec![
//             vec![(1, 10), (2, 3)],                 // Node 0
//             vec![(2, 1), (3, 2)],                  // Node 1
//             vec![(1, 4), (3, 8), (4, 2)],          // Node 2
//             vec![(4, 7)],                          // Node 3
//             vec![(3, 9)],                          // Node 4
//         ]);
//         let start_time = Instant::now();
//         let (cost, path) = parallel_bi_dijkstra(graph, 0, 4);
//         let duration = start_time.elapsed();
//         println!("Test Complex Graph - Time elapsed: {:?}", duration);
//         assert_eq!(cost, 5);  // Shortest path cost: 5
//         assert_eq!(path, vec![0, 2, 4]);  // Shortest path: 0 -> 2 -> 4
//     }

//     #[test]
//     fn test_very_complex_graph() {
//         let graph = Arc::new(vec![
//             vec![(1, 5), (2, 1), (3, 10)],     // Node 0
//             vec![(0, 5), (2, 3), (4, 1)],      // Node 1
//             vec![(0, 1), (1, 3), (3, 4), (4, 8)],  // Node 2
//             vec![(0, 10), (2, 4), (4, 2)],     // Node 3
//             vec![(1, 1), (2, 8), (3, 2), (5, 6)],  // Node 4
//             vec![(4, 6)],                      // Node 5
//         ]);
//         let start_time = Instant::now();
//         let (cost, path) = parallel_bi_dijkstra(graph, 0, 5);
//         let duration = start_time.elapsed();
//         println!("Test Very Complex Graph - Time elapsed: {:?}", duration);
//         assert_eq!(cost, 11);  // Shortest path cost: 11
//         assert_eq!(path, vec![0, 2, 1, 4, 5]);  // Shortest path: 0 -> 2 -> 1 -> 4 -> 5
//     }


// }