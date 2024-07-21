use std::cmp::Ordering;
use std::collections::BinaryHeap;
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

pub fn sequential_dijkstra(graph: &[Vec<(usize, usize)>], start: usize, goal: usize) -> (usize, Vec<usize>) {
    if start == goal {
        return (0, vec![start]);
    }

    let mut dist = vec![usize::MAX; graph.len()];
    let mut heap = BinaryHeap::new();
    let mut prev = vec![None; graph.len()];

    dist[start] = 0;
    heap.push(State { cost: 0, position: start });

    while let Some(State { cost, position }) = heap.pop() {
        if position == goal {
            let path = reconstruct_path(goal, &prev);
            return (cost, path);
        }

        if cost > dist[position] {
            continue;
        }

        for &(neighbor, weight) in &graph[position] {
            let next_cost = cost.saturating_add(weight);
            if next_cost < dist[neighbor] {
                dist[neighbor] = next_cost;
                heap.push(State { cost: next_cost, position: neighbor });
                prev[neighbor] = Some(position);
            }
        }
    }

    (usize::MAX, Vec::new())
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

fn main() {
    let graph = vec![
        vec![(1, 7), (2, 9), (5, 14)], // edges from node 0
        vec![(0, 7), (2, 10), (3, 15)], // edges from node 1
        vec![(0, 9), (1, 10), (3, 11), (5, 2)], // edges from node 2
        vec![(1, 15), (2, 11), (4, 6)], // edges from node 3
        vec![(3, 6), (5, 9)], // edges from node 4
        vec![(0, 14), (2, 2), (4, 9)], // edges from node 5
    ];

    let (cost, path) = sequential_dijkstra(&graph, 0, 4);
    println!("Cost: {}, Path: {:?}", cost, path);
}


fn generate_large_graph(nodes: usize, edges_per_node: usize, max_weight: usize, seed: u64) -> Vec<Vec<(usize, usize)>> {
    let mut rng = StdRng::seed_from_u64(seed);
    let weight_dist = Uniform::from(1..=max_weight);

    let mut graph = vec![Vec::new(); nodes];
    for node in 0..nodes {
        // Each node will have a random number of edges up to `edges_per_node`
        let num_edges = rng.gen_range(1..=edges_per_node);
        for _ in 0..num_edges {
            let target = rng.gen_range(0..nodes);
            let weight = weight_dist.sample(&mut rng);
            // Ensure no self-loops or duplicate edges
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
        let (cost, path) = sequential_dijkstra(&graph, 0, 2);
        let duration = start_time.elapsed();
        println!("Test Simple Graph - Time elapsed: {:?}", duration);
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
        let (cost, path) = sequential_dijkstra(&graph, 0, 3);
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
        let (cost, path) = sequential_dijkstra(&graph, 0, 4);
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
        let (cost, path) = sequential_dijkstra(&graph, 0, 5);
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
        let goal = Some(nodes - nodes/3); // Assuming we want to find path from node 0 to the last node
        let start_time = std::time::Instant::now();
        let (cost, path) = sequential_dijkstra(&graph, start, nodes - 1);
        let duration = start_time.elapsed();

        println!("Test Huge Graph - Time elapsed: {:?}, Cost: {}, Path Length: {}", duration, cost, path.len());
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
        let (cost, path) = sequential_dijkstra(&graph, 0, 999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Simple Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }

    #[test]
    fn test_large_disconnected_graph() {
        let graph = generate_random_graph(1000, 3000);
        let start_time = Instant::now();
        let (cost, path) = sequential_dijkstra(&graph, 0, 999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Disconnected Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }

    #[test]
    fn test_large_larger_graph() {
        let graph = generate_random_graph(1000, 8000);
        let start_time = Instant::now();
        let (cost, path) = sequential_dijkstra(&graph, 0, 999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Larger Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }

    #[test]
    fn test_large_complex_graph() {
        let graph = generate_random_graph(2000, 10000);
        let start_time = Instant::now();
        let (cost, path) = sequential_dijkstra(&graph, 0, 1999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Complex Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }

    #[test]
    fn test_large_very_complex_graph() {
        let graph = generate_random_graph(2000, 15000);
        let start_time = Instant::now();
        let (cost, path) = sequential_dijkstra(&graph, 0, 1999);
        let duration = start_time.elapsed();
        println!("Bi-Test Large Very Complex Graph - Time elapsed: {:?}", duration);
        println!("Cost: {}, Path length: {}", cost, path.len());
    }

}

/*
successes:

---- standard_dijkstra::tests::test_simple_graph stdout ----
Test Simple Graph - Time elapsed: 260.708µs

---- standard_dijkstra::tests::test_complex_graph stdout ----
Test Complex Graph - Time elapsed: 338.416µs

---- standard_dijkstra::tests::test_larger_graph stdout ----
Test Larger Graph - Time elapsed: 291.708µs

---- standard_dijkstra::tests::test_very_complex_graph stdout ----
Test Very Complex Graph - Time elapsed: 98.209µs

---- standard_dijkstra::tests::test_disconnected_graph stdout ----
Test Disconnected Graph - Time elapsed: 334.25µs

---- standard_dijkstra::tests::test_huge_graph stdout ----
Test Huge Graph - Time elapsed: 252.426083ms, Cost: 54, Path Length: 13


 */