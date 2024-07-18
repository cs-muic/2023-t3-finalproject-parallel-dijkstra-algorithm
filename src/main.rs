// FOR ADDITIONAL BENCHMARK TEST

use my_dijkstra_crate::{sequential_dijkstra, bidirectional_dijkstra, parallel_bi_dijkstra};
use std::time::Instant;

fn is_valid_path(graph: &Vec<Vec<(usize, usize)>>, path: &Vec<usize>, expected_cost: usize) -> bool {
    if path.is_empty() {
        return false;
    }

    let mut total_cost = 0;
    for window in path.windows(2) {
        if let Some(&(next, weight)) = graph[window[0]].iter().find(|&&(next, _)| next == window[1]) {
            total_cost += weight;
        } else {
            return false;
        }
    }
    total_cost == expected_cost
}

fn main() {
    // Define the number of nodes for the large graphs
    let num_nodes = 100;
    let num_large_nodes = 1000;  // Define the number of nodes for the super large graph
    let num_huge_nodes = 5000;  // Define the number of nodes for the super huge graph

    // Initialize a very large and dense graph with proper indices
    let dense_graph = (0..num_nodes).map(|i| {
        (0..num_nodes).filter(|&j| j != i).map(|j| (j, (i + j) % 10 + 1)).collect::<Vec<_>>()
    }).collect::<Vec<_>>();

    // Initialize a super large and dense graph
    let super_large_dense_graph = (0..num_large_nodes).map(|i| {
        (0..num_large_nodes).filter(|&j| j != i).map(|j| (j, (i + j) % 10 + 1)).collect::<Vec<_>>()
    }).collect::<Vec<_>>();

    // Initialize a super huge and dense graph
    let super_huge_dense_graph = (0..num_huge_nodes).map(|i| {
        (0..num_huge_nodes).filter(|&j| j != i).map(|j| (j, (i + j) % 10 + 1)).collect::<Vec<_>>()
    }).collect::<Vec<_>>();

    // Initialize a mega huge and dense graph
    let mega_huge_dense_graph = (0..10000).map(|i| {
        (0..10000).filter(|&j| j != i).map(|j| (j, (i + j) % 10 + 1)).collect::<Vec<_>>()
    }).collect::<Vec<_>>();

    // Define multiple test cases
    let test_cases = vec![
        (
            "Simple Path",
            vec![
                vec![(1, 2)],  // Node 0 connections
                vec![(2, 2)],  // Node 1 connections
                vec![(3, 2)],  // Node 2 connections
                vec![]         // Node 3 has no outgoing edges
            ],
            0,
            3,
        ),
        (
            "Graph with Cycles",
            vec![
                vec![(1, 2), (2, 1)],  // Node 0 connections
                vec![(3, 1)],          // Node 1 connections
                vec![(1, 2), (3, 2)],  // Node 2 connections
                vec![]                 // Node 3 has no outgoing edges
            ],
            0,
            3,
        ),
        (
            "Medium-Sized Sparse Graph",
            (0..500).map(|i| {
                (0..500).filter(|&j| j != i).take(5).map(|j| (j, (i + j) % 10 + 1)).collect::<Vec<_>>()
            }).collect::<Vec<_>>(),
            0,
            499,
        ),
        (
            "Medium-Sized Dense Graph with Close Start and Goal",
            (0..500).map(|i| {
                (0..500).filter(|&j| j != i).map(|j| (j, (i + j) % 10 + 1)).collect::<Vec<_>>()
            }).collect::<Vec<_>>(),
            0,
            10,
        ),
        (
            "Very Complex Graph",
            vec![
                vec![(1, 5), (2, 1), (3, 10)],     // Node 0
                vec![(0, 5), (2, 3), (4, 1)],      // Node 1
                vec![(0, 1), (1, 3), (3, 4), (4, 8)],  // Node 2
                vec![(0, 10), (2, 4), (4, 2)],     // Node 3
                vec![(1, 1), (2, 8), (3, 2), (5, 6)],  // Node 4
                vec![(4, 6)],                      // Node 5
            ],
            0,
            5,
        ),
        (
            "Large Graph with Multiple Paths",
            vec![
                vec![(1, 1), (5, 3)],               // Node 0 connections
                vec![(2, 2), (4, 3)],               // Node 1 connections
                vec![(3, 4)],                       // Node 2 connections
                vec![(4, 1), (6, 1)],               // Node 3 connections
                vec![(7, 5)],                       // Node 4 connections
                vec![(6, 2)],                       // Node 5 connections
                vec![(7, 1)],                       // Node 6 connections
                vec![]                              // Node 7 has no outgoing edges
            ],
            0,
            7,
        ),
        (
            "Very Large and Dense Graph",
            dense_graph,
            0,
            99,
        ),
        (
            "Super Large and Dense Graph",
            super_large_dense_graph,
            0,
            700,
        ),
        (
            "Super Huge and Dense Graph",
            super_huge_dense_graph,
            0,
            2999,
        ),
        (
            "Mega Huge and Dense Graph",
            mega_huge_dense_graph,
            0,
            9999,
        ),
    ];

    // Run each test case
    for (name, adj_list, start, goal) in test_cases {
        println!("Test Case: {}", name);

        // Benchmark sequential Dijkstra
        let start_time = Instant::now();
        let (sequential_cost, sequential_path) = sequential_dijkstra(&adj_list, start, Some(goal));
        let sequential_duration = start_time.elapsed();
        println!("Sequential Dijkstra: cost = {:?}, path = {:?}, duration = {:?}", sequential_cost, sequential_path, sequential_duration);

        // Benchmark bidirectional Dijkstra
        let start_time = Instant::now();
        let (bidirectional_cost, bidirectional_path) = bidirectional_dijkstra(&adj_list, start, goal);
        let bidirectional_duration = start_time.elapsed();
        println!("Bidirectional Dijkstra: cost = {:?}, path = {:?}, duration = {:?}", bidirectional_cost, bidirectional_path, bidirectional_duration);

         // Benchmark bidirectional Dijkstra
         /* 
         let start_time = Instant::now();
         let (par_bidirectional_cost, par_bidirectional_path) = parallel_bi_dijkstra(&adj_list, start, goal);
         let par_bidirectional_duration = start_time.elapsed();
         println!("Parallel Bidirectional Dijkstra: cost = {:?}, path = {:?}, duration = {:?}", par_bidirectional_cost, par_bidirectional_path, par_bidirectional_duration);
*/
        // Ensure both algorithms produce the same cost
        assert_eq!(sequential_cost, bidirectional_cost, "Costs do not match for {}", name);

        // Ensure both paths are valid
        if sequential_cost != usize::MAX {
            assert!(is_valid_path(&adj_list, &sequential_path, sequential_cost), "Sequential path is not valid for {}", name);
            assert!(is_valid_path(&adj_list, &bidirectional_path, bidirectional_cost), "Bidirectional path is not valid for {}", name);
        } else {
            assert!(sequential_path.is_empty(), "Sequential path should be empty for {}", name);
            assert!(bidirectional_path.is_empty(), "Bidirectional path should be empty for {}", name);
        }

        println!();  // Add a blank line between test cases
    }
}
