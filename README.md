# Concurrent & Parallel Dijkstra Project

The project aims to utilize concurrency and parallelism to speed up Dijkstra's Shortest Path Algorithm. It includes four variations of Dijkstra's algorithm, each representing a step improvement in performance:

1. **Unidirectional/Standard Dijkstra**
2. **Bidirectional Dijkstra**
3. **Parallel Dijkstra**
4. **Parallel Bidirectional Dijkstra**

## Variations

1. **Unidirectional/Standard Dijkstra**: The classic Dijkstra algorithm that finds the shortest path from a single source node to all other nodes in the graph.
2. **Bidirectional Dijkstra**: An optimized version that simultaneously searches from the source and target nodes, meeting in the middle to reduce the search space.
3. **Parallel Dijkstra**: A parallel implementation of the standard Dijkstra algorithm, utilizing multiple threads to speed up the search process.
4. **Parallel Bidirectional Dijkstra**: Combines bidirectional search with parallelism to further enhance performance.

## Running Tests

There are two sources of tests in the project: in-file tests and `main.rs` tests.

### In-file Tests

To run the in-file tests, use the following command:

```sh
cargo test --release -- --nocapture 
```

### Main.rs Tests

To run the `main.rs`, use the following command:

```sh
cargo test --release -- --nocapture
```
