
use super::math_helper::Vec2;

#[derive(Debug,Clone, Copy, PartialEq)]
pub enum NodeState {
    Clear,
    Visited,
    Closed,
    Solution
}
#[derive(Debug, Clone)]
struct NavNode {
    position: Vec2,
    connections: Vec<(usize, f32)>,
    ancestor_node: usize,
    g_value : f32,
    f_value : f32,
    state : NodeState

}

impl NavNode {
    pub fn new(position: Vec2) -> Self {
        Self {
            position,
            connections: Vec::new(),
            ancestor_node : 0,
            g_value : 0.0,
            f_value : 0.0,
            state : NodeState::Clear
        }
    }

    pub fn reset(&mut self) {
        self.state = NodeState::Clear;
    }
}


/// The graph structure that may be used for navigation.
pub struct NavGraph {
    nodes: Vec<NavNode>,
    links: Vec<(usize,usize)>,
}

impl NavGraph {
    /// Generates a new nav graph.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            links: Vec::new(),
        }
    }

    /// Checks if a link handed over is a solution link.
    fn is_solution_link(&self, link : &(usize, usize)) -> bool {
        (self.nodes[link.0].state == NodeState::Solution) && (self.nodes[link.1].state == NodeState::Solution)
    }

    pub fn get_all_solution_links(&self) -> impl Iterator<Item = &(usize, usize)>  {
        self.links.iter().filter(|link| self.is_solution_link(*link))
    }

    pub fn get_all_simple_links(&self) -> impl Iterator<Item = &(usize, usize)>  {
        self.links.iter().filter(|link| !self.is_solution_link(*link))
    }


    /// Finds the nearest node to the indicated position.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// use astar_lib::a_star::NavGraph;
    /// let mut graph = NavGraph::new();
    /// let p0 = graph.add_node(Vec2::new(0.0, 0.0));
    /// let index = graph.find_nearest_node(Vec2::new(0.1, 0.0))
    /// ```
    pub fn find_nearest_node(&self, position : &Vec2) -> usize {
        let mut min_dist = f32::MAX;
        let mut best_index = 0usize;

        for (index, node) in self.nodes.iter().enumerate() {
            let dist =  node.position.dist_to(position);
            if dist < min_dist {
                min_dist = dist;
                best_index = index;
            }
        }
        best_index
    }

    /// Adds a position to the nav graph and returns a handle index that may be used for
    /// connecting the nodes.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// use astar_lib::a_star::NavGraph;
    /// let mut graph = NavGraph::new();
    /// let p0 = graph.add_node(Vec2::new(0.0, 0.0));
    /// ```
    pub fn add_node(&mut self, position: Vec2) -> usize {
        let ret_val = self.nodes.len();
        self.nodes.push(NavNode::new(position));
        ret_val
    }

    /// Gets the position and the state of a node used for rendering later on.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// use astar_lib::a_star::NavGraph;
    /// let mut graph = NavGraph::new();
    /// let p0 = graph.add_node(Vec2::new(0.0, 0.0));
    /// let (position, state) = graph.ask_node(p0);
    /// ```
    pub fn ask_node(&self, handle: usize) -> (Vec2, NodeState) {
        let node = &self.nodes[handle];
        (node.position, node.state)
    }

    /// Connects two graph nodes with indicated indices.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// use astar_lib::a_star::NavGraph;
    /// let mut graph = NavGraph::new();
    /// let p0 = graph.add_node(Vec2::new(0.0, 0.0));
    /// let p1 = graph.add_node(Vec2::new(1.0, 1.0));
    /// graph.connect_nodes(p0, p1);
    /// ```
    pub fn connect_nodes(&mut self, node1: usize, node2: usize) {
        assert!(node1 < self.nodes.len(), "Node 1 does not exist");
        assert!(node2 < self.nodes.len(), "Node 2 does not exist");

        let dist = self.nodes[node1].position.dist_to(&self.nodes[node2].position);
        self.nodes[node1].connections.push((node2, dist));
        self.nodes[node2].connections.push((node1, dist));
        self.links.push((node1, node2));
    }

    /// Asks for the link list of the graph, returns the indices.
    pub fn get_link_list(&self) -> &Vec<(usize, usize)> {
        &self.links
    }

    fn reset_graph_search(&mut self) {
        for node in self.nodes.iter_mut() {
            node.reset();
        }
    }


    fn get_path(&mut self,  start_index : usize, destination_index : usize) -> Vec<usize> {
        let mut path : Vec<usize> = Vec::new();
        let mut scan = destination_index;

        while scan != start_index
        {
            path.push(scan);
            self.nodes[scan].state = NodeState::Solution;
            scan = self.nodes[scan].ancestor_node;
        }
        self.nodes[scan].state = NodeState::Solution;
        path.push(scan);
        path.reverse();
        path
    }


    /// Does the real search from the start point to the end point of the graph. This is the real search operation.
    /// # Parameters:
    /// * start: The start point to start searching for,
    /// * end: The end point on the search.
    /// # Returns
    /// If a path could be found it returns the positions of the path.
    ///
    /// # Example:
    ///
    ///  ```
    ///  use astar_lib::math_helper::Vec2;
    ///  use astar_lib::a_star::NavGraph;
    ///  let mut graph = NavGraph::new();
    ///  let p0 = graph.add_node(Vec2::new(0.0, 0.0));
    ///  let p1 = graph.add_node(Vec2::new(0.5, 0.5));
    ///  let p2 = graph.add_node(Vec2::new(1.0, 0.0));
    ///  let p3 = graph.add_node(Vec2::new(1.0, 1.0));
    ///  let p4 = graph.add_node(Vec2::new(0.1, 0.0));
    ///  graph.connect_nodes(p0, p1);
    ///  graph.connect_nodes(p1, p2);
    ///  graph.connect_nodes(p0, p2);
    ///  graph.connect_nodes(p1, p4);
    ///  graph.connect_nodes(p4, p3);
    ///  graph.connect_nodes(p2, p3);
    ///
    ///  let result = graph.search_graph(p0, p3);
    ///
    ///  if let Some(result) = result {
    ///      for pos in result.iter() {
    ///          println!("{:?}", pos);
    ///       } }
    ///  ```
    pub fn search_graph(&mut self, start_index: usize, destination_index :usize) -> Option<Vec<usize>> {
        self.reset_graph_search();
        let dest_point = self.nodes[destination_index].position;
        let mut todo_list : Vec<usize> = Vec::new();

        self.nodes[start_index].state = NodeState::Visited;
        todo_list.push(start_index);

        loop {

            // In this case there is no path we return none.
            let (best_index, best_candidate) = todo_list.iter().enumerate().min_by(|a, b|
                self.nodes[*a.1].f_value.total_cmp( &self.nodes[*b.1].f_value))?;
            let best_candidate = *best_candidate;
            todo_list.swap_remove(best_index);

            self.nodes[best_candidate].state = NodeState::Closed;

            if best_candidate == destination_index {
                return Some(self.get_path( start_index, destination_index));
            }

            let connection_count = self.nodes[best_candidate].connections.len();
            let root_g_value = self.nodes[best_candidate].g_value;

            for partner in 0..connection_count {
                let (global_index, distance) = self.nodes[best_candidate].connections[partner];
                let partner_node = &mut self.nodes[global_index];

                match partner_node.state {
                    NodeState::Clear => {
                        partner_node.state = NodeState::Visited;
                        partner_node.ancestor_node = best_candidate;
                        partner_node.g_value = root_g_value + distance;
                        partner_node.f_value = partner_node.g_value + partner_node.position.dist_to(&dest_point);
                        todo_list.push(global_index);
                    },
                    NodeState::Visited => {
                        let new_g_value = root_g_value + distance;
                        if new_g_value < partner_node.g_value {
                            partner_node.g_value = new_g_value;
                            partner_node.f_value = new_g_value + partner_node.position.dist_to(&dest_point);
                            partner_node.ancestor_node = best_candidate;
                        }
                    }
                    NodeState::Closed => {},
                    NodeState::Solution => {panic!("Case should not happen")}
                }
            }

        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base_test()
    {
        let mut graph = NavGraph::new();

        let p0 = graph.add_node(Vec2::new(0.0, 0.0));
        let p1 = graph.add_node(Vec2::new(0.5, 0.5));
        let p2 = graph.add_node(Vec2::new(1.0, 0.0));
        let p3 = graph.add_node(Vec2::new(1.0, 1.0));
        let p4 = graph.add_node(Vec2::new(0.1, 0.0));
        let p5 = graph.add_node(Vec2::new(2.0, 2.0));

        graph.connect_nodes(p0, p1);
        graph.connect_nodes(p1, p2);
        graph.connect_nodes(p0, p2);
        graph.connect_nodes(p1, p4);
        graph.connect_nodes(p4, p3);
        graph.connect_nodes(p2, p3);

        let result = graph.search_graph(p0, p3);
        assert!(result.is_some());

        let result = result.unwrap();
        assert_eq!(result, [0,2,3]);
        
        for (source, destination) in graph.get_all_simple_links() {
            println!("{:?} -> {:?}", source, destination);
        }



        let result = graph.search_graph(p0,p5);
        assert!(result.is_none(), "There should not be a solution!");

    }
}
