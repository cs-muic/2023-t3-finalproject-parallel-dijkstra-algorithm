extern crate time;
use time::Instant;

use my_dijkstra_crate::{dijkstra::sequential_dijkstra, bidirectional::bidirectional_dijkstra};

fn main() {
    let graph = vec![
        vec![(1, 2), (2, 4)],  // Node 0 is connected to Node 1 (cost 2) and Node 2 (cost 4)
        vec![(2, 1)],          // Node 1 is connected to Node 2 (cost 1)
        vec![]                 // Node 2 has no outgoing edges
    ];

    let start = 0;
    let goal = 2;

    // Benchmark sequential Dijkstra
    let start_time = Instant::now();
    let result_sequential = sequential_dijkstra(&graph, start);
    let elapsed_time_sequential = start_time.elapsed();

    // Benchmark bidirectional Dijkstra
    let start_time = Instant::now();
    let result_bidirectional = bidirectional_dijkstra(&graph, start, goal);
    let elapsed_time_bidirectional = start_time.elapsed();

    // Extract the goal node result from the sequential version to compare with bidirectional
    let sequential_goal_result = result_sequential[goal];

    println!("Sequential Dijkstra Result for Goal: {:?}", sequential_goal_result);
    println!("Bidirectional Dijkstra Result: {:?}", result_bidirectional);
    
    println!("Sequential Dijkstra Time: {:?}", elapsed_time_sequential);
    println!("Bidirectional Dijkstra Time: {:?}", elapsed_time_bidirectional);
}
