use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

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

pub fn bidirectional_dijkstra(
    adj_list: &[Vec<(usize, usize)>],
    rev_adj_list: &[Vec<(usize, usize)>],
    start: usize,
    goal: usize
) -> Option<(usize, Vec<usize>)> {
    let mut dist_s = vec![usize::MAX; adj_list.len()];
    let mut dist_t = vec![usize::MAX; adj_list.len()];
    let mut heap_s = BinaryHeap::new();
    let mut heap_t = BinaryHeap::new();
    let mut prev_s = vec![None; adj_list.len()];
    let mut prev_t = vec![None; adj_list.len()];

    dist_s[start] = 0;
    dist_t[goal] = 0;
    heap_s.push(State { cost: 0, position: start });
    heap_t.push(State { cost: 0, position: goal });

    let mut best_path = usize::MAX;
    let mut meeting_point = None;

    while let (Some(State { cost: cost_s, position: pos_s }), Some(State { cost: cost_t, position: pos_t })) = (heap_s.peek(), heap_t.peek()) {
        if cost_s + cost_t >= best_path {
            break;
        }

        if let Some(State { cost: cost_s, position: pos_s }) = heap_s.pop() {
            if dist_t[pos_s] != usize::MAX {
                let total_cost = cost_s + dist_t[pos_s];
                if total_cost < best_path {
                    best_path = total_cost;
                    meeting_point = Some(pos_s);
                }
            }

            for &(neighbor, weight) in &adj_list[pos_s] {
                let next = State { cost: cost_s + weight, position: neighbor };
                if next.cost < dist_s[neighbor] {
                    dist_s[neighbor] = next.cost;
                    heap_s.push(next);
                    prev_s[neighbor] = Some(pos_s);
                }
            }
        }

        if let Some(State { cost: cost_t, position: pos_t }) = heap_t.pop() {
            if dist_s[pos_t] != usize::MAX {
                let total_cost = cost_t + dist_s[pos_t];
                if total_cost < best_path {
                    best_path = total_cost;
                    meeting_point = Some(pos_t);
                }
            }

            for &(neighbor, weight) in &rev_adj_list[pos_t] {
                let next = State { cost: cost_t + weight, position: neighbor };
                if next.cost < dist_t[neighbor] {
                    dist_t[neighbor] = next.cost;
                    heap_t.push(next);
                    prev_t[neighbor] = Some(pos_t);
                }
            }
        }
    }

    if best_path == usize::MAX {
        return None;
    }

    let meeting_point = meeting_point.unwrap();

    let mut path = Vec::new();
    let mut current = meeting_point;
    while let Some(prev) = prev_s[current] {
        path.push(current);
        current = prev;
    }
    path.push(start);
    path.reverse();

    current = meeting_point;
    while let Some(prev) = prev_t[current] {
        current = prev;
        path.push(current);
    }

    Some((best_path, path))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bidirectional_same_start_goal() {
        let adj_list = vec![
            vec![(1, 2), (2, 4)],
            vec![(2, 1)],
            vec![(3, 1)],
            vec![]
        ];
        let rev_adj_list = vec![
            vec![],
            vec![(0, 2)],
            vec![(0, 4), (1, 1)],
            vec![(2, 1)]
        ];
        assert_eq!(bidirectional_dijkstra(&adj_list, &rev_adj_list, 0, 0), Some((0, vec![0])));
    }

    #[test]
    fn test_disconnected_graph() {
        let adj_list = vec![
            vec![(1, 2)],
            vec![],
            vec![(3, 1)],
            vec![]
        ];
        let rev_adj_list = vec![
            vec![],
            vec![(0, 2)],
            vec![],
            vec![(2, 1)]
        ];
        assert_eq!(bidirectional_dijkstra(&adj_list, &rev_adj_list, 0, 3), None);
    }

    #[test]
    fn test_simple_path() {
        let adj_list = vec![
            vec![(1, 2)],
            vec![(2, 2)],
            vec![(3, 2)],
            vec![]
        ];
        let rev_adj_list = vec![
            vec![],
            vec![(0, 2)],
            vec![(1, 2)],
            vec![(2, 2)]
        ];
        assert_eq!(bidirectional_dijkstra(&adj_list, &rev_adj_list, 0, 3), Some((6, vec![0, 1, 2, 3])));
    }

    #[test]
    fn test_graph_with_cycles() {
        let adj_list = vec![
            vec![(1, 2), (2, 1)],
            vec![(3, 1)],
            vec![(1, 2), (3, 2)],
            vec![]
        ];
        let rev_adj_list = vec![
            vec![],
            vec![(0, 2), (2, 2)],
            vec![(0, 1)],
            vec![(1, 1), (2, 2)]
        ];
        assert_eq!(bidirectional_dijkstra(&adj_list, &rev_adj_list, 0, 3), Some((3, vec![0, 2, 3])));
    }

    #[test]
    fn test_bidirectional_larger_graph() {
        let adj_list = vec![
            vec![(1, 1), (2, 4)],
            vec![(3, 1)],
            vec![(3, 1)],
            vec![(4, 1)],
            vec![]
        ];
        let rev_adj_list = vec![
            vec![],
            vec![(0, 1)],
            vec![(0, 4)],
            vec![(1, 1), (2, 1)],
            vec![(3, 1)]
        ];
        assert_eq!(bidirectional_dijkstra(&adj_list, &rev_adj_list, 0, 4), Some((3, vec![0, 1, 3, 4])));
    }
}

fn main() {
    let adj_list = vec![
        vec![(1, 2), (2, 4)],
        vec![(2, 1)],
        vec![(3, 1)],
        vec![]
    ];
    let rev_adj_list = vec![
        vec![],
        vec![(0, 2)],
        vec![(0, 4), (1, 1)],
        vec![(2, 1)]
    ];
    let result = bidirectional_dijkstra(&adj_list, &rev_adj_list, 0, 3);
    match result {
        Some((cost, path)) => println!("Shortest path cost: {}, Path: {:?}", cost, path),
        None => println!("No path found"),
    }
}
