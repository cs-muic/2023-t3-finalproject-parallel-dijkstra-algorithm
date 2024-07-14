use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: usize,
    position: usize,
}

// The priority queue depends on `Ord`.
impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

// `PartialOrd` needs to be implemented as well.
impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn relax(
    heap: &mut BinaryHeap<State>,
    dist: &mut Vec<usize>,
    other_dist: &Vec<usize>,
    shortest_path: &mut usize,
    position: usize,
    cost: usize,
    neighbors: &[(usize, usize)]
) {
    for &(next, weight) in neighbors {
        if let Some(next_cost) = cost.checked_add(weight) {
            if next_cost < dist[next] {
                dist[next] = next_cost;
                heap.push(State { cost: next_cost, position: next });
                if other_dist[next] != usize::MAX {
                    if let Some(potential_shortest) = next_cost.checked_add(other_dist[next]) {
                        *shortest_path = (*shortest_path).min(potential_shortest);
                    }
                }
            }
        }
    }
}

pub fn bidirectional_dijkstra(graph: &Vec<Vec<(usize, usize)>>, start: usize, goal: usize) -> Option<usize> {
    if start == goal {
        return Some(0);
    }

    let mut dist_forward: Vec<_> = (0..graph.len()).map(|_| usize::MAX).collect();
    let mut dist_backward: Vec<_> = (0..graph.len()).map(|_| usize::MAX).collect();
    let mut heap_forward = BinaryHeap::new();
    let mut heap_backward = BinaryHeap::new();

    dist_forward[start] = 0;
    dist_backward[goal] = 0;
    heap_forward.push(State { cost: 0, position: start });
    heap_backward.push(State { cost: 0, position: goal });

    let mut shortest_path = usize::MAX;

    while !heap_forward.is_empty() && !heap_backward.is_empty() {
        if let Some(State { cost: cost_f, position: pos_f }) = heap_forward.pop() {
            if cost_f <= dist_forward[pos_f] {
                relax(&mut heap_forward, &mut dist_forward, &dist_backward, &mut shortest_path, pos_f, cost_f, &graph[pos_f]);
            }
        }

        if let Some(State { cost: cost_b, position: pos_b }) = heap_backward.pop() {
            if cost_b <= dist_backward[pos_b] {
                let mut neighbors: Vec<(usize, usize)> = vec![];
                for (u, v) in graph.iter().enumerate() {
                    for &(next, weight) in v {
                        if next == pos_b {
                            neighbors.push((u, weight));
                        }
                    }
                }
                relax(&mut heap_backward, &mut dist_backward, &dist_forward, &mut shortest_path, pos_b, cost_b, &neighbors);
            }
        }

        let forward_min_cost = heap_forward.peek().map_or(usize::MAX, |s| s.cost);
        let backward_min_cost = heap_backward.peek().map_or(usize::MAX, |s| s.cost);
        if forward_min_cost != usize::MAX && backward_min_cost != usize::MAX {
            if let Some(combined_cost) = forward_min_cost.checked_add(backward_min_cost) {
                if combined_cost >= shortest_path {
                    break;
                }
            }
        }
    }

    if shortest_path == usize::MAX {
        None
    } else {
        Some(shortest_path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bidirectional_dijkstra_simple_graph() {
        let graph = vec![
            vec![(1, 2), (2, 4)],  // Node 0 is connected to Node 1 (cost 2) and Node 2 (cost 4)
            vec![(2, 1)],          // Node 1 is connected to Node 2 (cost 1)
            vec![]                 // Node 2 has no outgoing edges
        ];
        let result = bidirectional_dijkstra(&graph, 0, 2);
        assert_eq!(result, Some(3));  // Shortest path: 0->1->2 (cost 3)
    }

    #[test]
    fn test_bidirectional_dijkstra_disconnected_graph() {
        let graph = vec![
            vec![(1, 2)],  // Node 0 is connected to Node 1 (cost 2)
            vec![],        // Node 1 has no outgoing edges
            vec![]         // Node 2 is disconnected
        ];
        let result = bidirectional_dijkstra(&graph, 0, 2);
        assert_eq!(result, None);  // Node 2 is unreachable from Node 0
    }

    #[test]
    fn test_bidirectional_dijkstra_larger_graph() {
        let graph = vec![
            vec![(1, 1), (2, 4), (3, 7)],  // Node 0 connections
            vec![(3, 2)],                  // Node 1 connections
            vec![(3, 1)],                  // Node 2 connections
            vec![]                         // Node 3 has no outgoing edges
        ];
        let result = bidirectional_dijkstra(&graph, 0, 3);
        assert_eq!(result, Some(3));  // Shortest path: 0->2->3 (cost 3)
    }

    #[test]
    fn test_bidirectional_dijkstra_complex_graph() {
        let graph = vec![
            vec![(1, 10), (2, 3)],                 // Node 0
            vec![(2, 1), (3, 2)],                  // Node 1
            vec![(1, 4), (3, 8), (4, 2)],          // Node 2
            vec![(4, 7)],                          // Node 3
            vec![(3, 9)],                          // Node 4
        ];
        let result = bidirectional_dijkstra(&graph, 0, 4);
        assert_eq!(result, Some(5));  // Shortest path: 0->2->4 (cost 5)
    }

    #[test]
    fn test_bidirectional_dijkstra_very_complex_graph() {
        let graph = vec![
            vec![(1, 5), (2, 1), (3, 10)],     // Node 0
            vec![(0, 5), (2, 3), (4, 1)],      // Node 1
            vec![(0, 1), (1, 3), (3, 4), (4, 8)],  // Node 2
            vec![(0, 10), (2, 4), (4, 2)],     // Node 3
            vec![(1, 1), (2, 8), (3, 2), (5, 6)],  // Node 4
            vec![(4, 6)],                      // Node 5
        ];
        let result = bidirectional_dijkstra(&graph, 0, 5);
        assert_eq!(result, Some(11));  // Shortest path: 0->2->3->4->5 (cost 11)
    }
}
