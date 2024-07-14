use my_dijkstra_crate::{Graph, sequential_dijkstra, bidirectional_dijkstra};

extern crate time;
use std::time::Instant;


fn main() {
    let adj_list = vec![
        vec![(1, 2), (2, 4), (3, 7)],  // Node 0 connections
        vec![(3, 1)],                  // Node 1 connections
        vec![(3, 1)],                  // Node 2 connections
        vec![]                         // Node 3 has no outgoing edges
    ];
    let rev_adj_list = vec![
        vec![],
        vec![(0, 2)],
        vec![(0, 4), (1, 1)],
        vec![(2, 1), (1, 1)]
    ];

    let start = 0;
    let goal = 3;

    // Benchmark sequential Dijkstra
    let start_time = Instant::now();
    let (sequential_cost, sequential_path) = sequential_dijkstra(&adj_list, start, Some(goal));
    let sequential_duration = start_time.elapsed();
    println!("Sequential Dijkstra: cost = {:?}, path = {:?}, duration = {:?}", sequential_cost, sequential_path, sequential_duration);

    // Benchmark bidirectional Dijkstra
    let start_time = Instant::now();
    let bidirectional_result = bidirectional_dijkstra(&adj_list, &rev_adj_list, start, goal);
    let bidirectional_duration = start_time.elapsed();
    println!("Bidirectional Dijkstra: result = {:?}, duration = {:?}", bidirectional_result, bidirectional_duration);
}
