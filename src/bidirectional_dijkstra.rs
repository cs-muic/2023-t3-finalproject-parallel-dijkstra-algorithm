use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use rand::{distributions::{Distribution, Uniform}, SeedableRng, rngs::StdRng, Rng};



#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: usize,
    position: usize,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // We reverse the order to make the priority queue a min-heap.
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// reverse the adjacency list for the backward search
fn reverse_adj_list(adj_list: &[Vec<(usize, usize)>]) -> Vec<Vec<(usize, usize)>> {
    let mut rev_adj_list = vec![Vec::new(); adj_list.len()];
    for (node, edges) in adj_list.iter().enumerate() {
        for &(neighbor, weight) in edges {
            rev_adj_list[neighbor].push((node, weight));
        }
    }
    rev_adj_list
}

fn relax(
    graph: &[Vec<(usize, usize)>],
    heap: &mut BinaryHeap<State>,
    dists: &mut Vec<usize>,
    prev: &mut Vec<Option<usize>>,
    u: usize,
    cost: usize,
) {
    for &(neighbor, weight) in &graph[u] {
        let next_cost = cost + weight;
        if next_cost < dists[neighbor] {
            dists[neighbor] = next_cost;
            heap.push(State { cost: next_cost, position: neighbor });
            prev[neighbor] = Some(u);
        }
    }
}

pub fn bidirectional_dijkstra(graph: &[Vec<(usize, usize)>], start: usize, goal: usize) -> (usize, Vec<usize>) {
    if start == goal {
        return (0, vec![start]);
    }

    let rev_graph = reverse_adj_list(graph);
    let mut dist_fwd = vec![usize::MAX; graph.len()];
    let mut dist_bwd = vec![usize::MAX; graph.len()];
    let mut heap_fwd = BinaryHeap::new();
    let mut heap_bwd = BinaryHeap::new();
    let mut visited_fwd = HashSet::new();
    let mut visited_bwd = HashSet::new();
    let mut prev_fwd = vec![None; graph.len()];
    let mut prev_bwd = vec![None; graph.len()];

    dist_fwd[start] = 0;
    dist_bwd[goal] = 0;
    heap_fwd.push(State { cost: 0, position: start });
    heap_bwd.push(State { cost: 0, position: goal });

    let mut best_cost = usize::MAX;
    let mut best_meeting_point = None;

    while !heap_fwd.is_empty() && !heap_bwd.is_empty() {
        if let Some(State { cost: cost_fwd, position: pos_fwd }) = heap_fwd.pop() {
            if visited_fwd.insert(pos_fwd) {
                relax(graph, &mut heap_fwd, &mut dist_fwd, &mut prev_fwd, pos_fwd, cost_fwd);

                if visited_bwd.contains(&pos_fwd) {
                    let total_cost = dist_fwd[pos_fwd] + dist_bwd[pos_fwd];
                    if total_cost < best_cost {
                        best_cost = total_cost;
                        best_meeting_point = Some(pos_fwd);
                    }
                }
            }
        }

        if let Some(State { cost: cost_bwd, position: pos_bwd }) = heap_bwd.pop() {
            if visited_bwd.insert(pos_bwd) {
                relax(&rev_graph, &mut heap_bwd, &mut dist_bwd, &mut prev_bwd, pos_bwd, cost_bwd);

                if visited_fwd.contains(&pos_bwd) {
                    let total_cost = dist_fwd[pos_bwd] + dist_bwd[pos_bwd];
                    if total_cost < best_cost {
                        best_cost = total_cost;
                        best_meeting_point = Some(pos_bwd);
                    }
                }
            }
        }

        let min_fwd_cost = heap_fwd.peek().map_or(usize::MAX, |state| state.cost);
        let min_bwd_cost = heap_bwd.peek().map_or(usize::MAX, |state| state.cost);
        if min_fwd_cost + min_bwd_cost >= best_cost {
            break;
        }
    }

    if let Some(meeting_point) = best_meeting_point {
        let path = reconstruct_path(meeting_point, &prev_fwd, &prev_bwd, start, goal);
        (best_cost, path)
    } else {
        (usize::MAX, vec![])
    }
}

fn reconstruct_path(meeting_point: usize, prev_fwd: &[Option<usize>], prev_bwd: &[Option<usize>], start: usize, goal: usize) -> Vec<usize> {
    let mut path = Vec::new();
    let mut current = Some(meeting_point);

    // from start to the meeting point
    while let Some(prev) = current {
        path.push(prev);
        if prev == start {
            break;
        }
        current = prev_fwd[prev];
    }
    path.reverse(); 

    //  from meeting point to goal
    current = prev_bwd[meeting_point];
    while let Some(prev) = current {
        path.push(prev);
        if prev == goal {
            break;
        }
        current = prev_bwd[prev];
    }
    
    path
}


   

fn generate_large_graph(nodes: usize, edges_per_node: usize, max_weight: usize, seed: u64) -> Vec<Vec<(usize, usize)>> {
    let mut rng = StdRng::seed_from_u64(seed);
    let weight_dist = Uniform::from(1..=max_weight);

    let mut graph = vec![Vec::new(); nodes];
    for node in 0..nodes {
        let num_edges = rng.gen_range(1..=edges_per_node);
        for _ in 0..num_edges {
            let target = rng.gen_range(0..nodes);
            let weight = weight_dist.sample(&mut rng);
            if target != node && !graph[node].iter().any(|&(t, _)| t == target) {
                graph[node].push((target, weight));
            }
        }
    }

    graph
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
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 2);
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
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 2);
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
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 3);
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
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 4);
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
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 5);
        let duration = start_time.elapsed();
        println!("Test Very Complex Graph - Time elapsed: {:?}", duration);
        assert_eq!(cost, 11);  // Shortest path cost: 11
        assert_eq!(path, vec![0, 2, 1, 4, 5]);  // Shortest path: 0 -> 2 -> 1 -> 4 -> 5
    }

    #[test]
    fn test_huge_graph() {
        let nodes = 100000; // 100,000 nodes
        let edges_per_node = 50; // Each node connects to 50 others, on average
        let max_weight = 100; // Maximum weight of 100
        let seed = 42; // Fixed seed for reproducibility

        let graph = generate_large_graph(nodes, edges_per_node, max_weight, seed);

        let start = 0;
        let goal = (nodes - nodes/3); // Assuming we want to find path from node 0 to the last node
        let start_time = std::time::Instant::now();
        let (cost, path) = bidirectional_dijkstra(&graph, start, goal);
        let duration = start_time.elapsed();

        println!("Test Huge Graph - Time elapsed: {:?}, Cost: {}, Path Length: {}", duration, cost, path.len());
    }
}

/*
successes:

---- bidirectional_dijkstra::tests::test_larger_graph stdout ----
Test Larger Graph - Time elapsed: 223.333µs

---- bidirectional_dijkstra::tests::test_simple_graph stdout ----
Test Simple Graph - Time elapsed: 71.084µs

---- bidirectional_dijkstra::tests::test_complex_graph stdout ----
Test Complex Graph - Time elapsed: 153.5µs

---- bidirectional_dijkstra::tests::test_disconnected_graph stdout ----
Test Disconnected Graph - Time elapsed: 260µs

---- bidirectional_dijkstra::tests::test_very_complex_graph stdout ----
Test Very Complex Graph - Time elapsed: 32.333µs

---- bidirectional_dijkstra::tests::test_huge_graph stdout ----
Test Huge Graph - Time elapsed: 14.525125ms, Cost: 37, Path Length: 8 */
