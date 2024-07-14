pub struct Graph {
    pub adj_list: Vec<Vec<(usize, usize)>>,
    pub rev_adj_list: Vec<Vec<(usize, usize)>>,
}

impl Graph {
    pub fn new(adj_list: Vec<Vec<(usize, usize)>>) -> Self {
        let mut rev_adj_list = vec![vec![]; adj_list.len()];
        for (u, neighbors) in adj_list.iter().enumerate() {
            for &(v, weight) in neighbors {
                rev_adj_list[v].push((u, weight));
            }
        }
        Graph {
            adj_list,
            rev_adj_list,
        }
    }
}
