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

pub fn bidirectional_dijkstra(graph: &[Vec<(usize, usize)>], start: usize, goal: usize) -> (usize, Vec<usize>) {
    if start == goal {
        return (0, vec![start]);
    }

    let rev_graph = reverse_adj_list(graph);
    let mut dist_fwd = vec![usize::MAX; graph.len()];
    let mut dist_bwd = vec![usize::MAX; graph.len()];
    let mut heap_fwd = BinaryHeap::new();
    let mut heap_bwd = BinaryHeap::new();
    let mut prev_fwd = vec![None; graph.len()];
    let mut prev_bwd = vec![None; graph.len()];

    dist_fwd[start] = 0;
    dist_bwd[goal] = 0;
    heap_fwd.push(State { cost: 0, position: start });
    heap_bwd.push(State { cost: 0, position: goal });

    let mut estimate = usize::MAX;
    let mut join_node = None;

    while !heap_fwd.is_empty() && !heap_bwd.is_empty() {
        let pos_fwd = heap_fwd.peek().unwrap().position;
        let pos_bwd = heap_bwd.peek().unwrap().position;
        let cost_fwd = heap_fwd.peek().unwrap().cost;
        let cost_bwd = heap_bwd.peek().unwrap().cost;

        if cost_fwd + cost_bwd >= estimate {
            break;
        }

        if cost_fwd < cost_bwd {
            let State { cost, position } = heap_fwd.pop().unwrap();
            if let Some((new_estimate, new_join_node)) = discover_nodes(&graph[position], position, &mut dist_fwd, &mut dist_bwd, &mut heap_fwd, &mut prev_fwd, estimate, join_node) {
                estimate = new_estimate;
                join_node = Some(new_join_node);
            }
        } else {
            let State { cost, position } = heap_bwd.pop().unwrap();
            if let Some((new_estimate, new_join_node)) = discover_nodes(&rev_graph[position], position, &mut dist_bwd, &mut dist_fwd, &mut heap_bwd, &mut prev_bwd, estimate, join_node) {
                estimate = new_estimate;
                join_node = Some(new_join_node);
            }
        }
    }

    if let Some(join) = join_node {
        let mut path_fwd = reconstruct_path(join, &prev_fwd);
        let mut path_bwd = reconstruct_path(join, &prev_bwd);
        path_bwd.reverse();
        path_fwd.pop(); // Avoid duplicate join node
        path_fwd.extend(path_bwd);
        (estimate, path_fwd)
    } else {
        (usize::MAX, Vec::new())
    }
}

fn discover_nodes(
    edges: &[(usize, usize)],
    node: usize,
    dist: &mut [usize],
    other_dist: &mut [usize],
    heap: &mut BinaryHeap<State>,
    prev: &mut [Option<usize>],
    mut estimate: usize,
    join_node: Option<usize>,
) -> Option<(usize, usize)> {
    let mut local_join_node = join_node;

    for &(neighbor, weight) in edges {
        let new_cost = dist[node].saturating_add(weight);
        if new_cost < dist[neighbor] {
            dist[neighbor] = new_cost;
            heap.push(State { cost: new_cost, position: neighbor });
            prev[neighbor] = Some(node);
        }
        if other_dist[neighbor] != usize::MAX {
            let total_cost = new_cost.saturating_add(other_dist[neighbor]);
            if total_cost < estimate {
                estimate = total_cost;
                local_join_node = Some(neighbor);
            }
        }
    }

    local_join_node.map(|join| (estimate, join))
}

fn reconstruct_path(meeting_point: usize, prev: &[Option<usize>]) -> Vec<usize> {
    let mut path = Vec::new();
    let mut current = Some(meeting_point);
    while let Some(node) = current {
        path.push(node);
        current = prev[node];
    }
    path.reverse();
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
    use rand::distributions::{Distribution, Uniform};
    use rand::rngs::StdRng;
    use rand::SeedableRng;
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
        println!("Bi-Test Simple Graph - Time elapsed: {:?}", duration);
        assert_eq!(cost, 3);  // Shortest path cost: 3
        assert_eq!(path, vec![0, 1, 2]);  // Shortest path: 0 -> 1 -> 2
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
        println!("Bi-Test Larger Graph - Time elapsed: {:?}", duration);
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
        println!("Bi-Test Complex Graph - Time elapsed: {:?}", duration);
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
        println!("Bi-Test Very Complex Graph - Time elapsed: {:?}", duration);
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

        println!("Bi-Test Huge Graph - Time elapsed: {:?}, Cost: {}, Path Length: {}", duration, cost, path.len());
    }

    fn generate_random_graph(nodes: usize, edges: usize) -> Vec<Vec<(usize, usize)>> {
        let mut graph = vec![Vec::new(); nodes];
        let mut rng = StdRng::seed_from_u64(42); // Seed for reproducibility
        let range = Uniform::from(0..nodes);
        let weight_range = Uniform::from(1..100); // Random weights between 1 and 100

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
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Simple Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }

    #[test]
    fn test_large_disconnected_graph() {
        let graph = generate_random_graph(1000, 3000);
        let start_time = Instant::now();
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Disconnected Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }

    #[test]
    fn test_large_larger_graph() {
        let graph = generate_random_graph(1000, 8000);
        let start_time = Instant::now();
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Larger Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }

    #[test]
    fn test_large_complex_graph() {
        let graph = generate_random_graph(2000, 10000);
        let start_time = Instant::now();
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 1999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Complex Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }

    #[test]
    fn test_large_very_complex_graph() {
        let graph = generate_random_graph(2000, 15000);
        let start_time = Instant::now();
        let (cost, path) = bidirectional_dijkstra(&graph, 0, 1999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Very Complex Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }
}


/*
successes:

---- bidirectional_dijkstra::tests::test_simple_graph stdout ----
Test Simple Graph - Time elapsed: 120.375µs

---- bidirectional_dijkstra::tests::test_complex_graph stdout ----
Test Complex Graph - Time elapsed: 376.291µs

---- bidirectional_dijkstra::tests::test_larger_graph stdout ----
Test Larger Graph - Time elapsed: 292.583µs

---- bidirectional_dijkstra::tests::test_very_complex_graph stdout ----
Test Very Complex Graph - Time elapsed: 42.292µs

---- bidirectional_dijkstra::tests::test_disconnected_graph stdout ----
Test Disconnected Graph - Time elapsed: 228.208µs

---- bidirectional_dijkstra::tests::test_huge_graph stdout ----
Test Huge Graph - Time elapsed: 329.195ms, Cost: 54, Path Length: 13
 */
