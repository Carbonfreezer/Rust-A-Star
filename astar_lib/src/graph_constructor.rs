//! Helper module to generate an interesting graph.

use crate::a_star::NavGraph;
use super::math_helper::{Line, Vec2};

/// The extension from 0 to [`EXTENSION`] we use for generating the nodes of the graph..
pub const EXTENSION: f32 = 1.0;

/// The radius we have around one point where no additional point should be added.
const EXCLUSION_RADIUS: f32 = 0.025;

/// The maximum length we can have for a line.
const MAX_LINE_LENGTH: f32 = 0.15;

/// The maximum number of iterations we make per attempt for link generation
const MAX_ITERATIONS: usize = 100000;


pub struct GraphConstructor {
    point_collection : Vec<Vec2>,
    point_pairing : Vec<(usize, usize)>,
}

impl GraphConstructor {
    
    /// Generates a new graph constructor
    ///
    /// # Example
    /// ```
    /// use astar_lib::graph_constructor::GraphConstructor;
    /// let mut constructor = GraphConstructor::new();
    /// ``` 

    pub fn new() -> GraphConstructor {
        GraphConstructor { point_collection: vec![],  point_pairing: vec![] }
    }


    /// Fills the point array with random points.
    /// 
    /// # Example
    /// ```
    /// use astar_lib::graph_constructor::GraphConstructor;
    /// let mut constructor = GraphConstructor::new();
    /// constructor.add_random_points(1000);
    /// ``` 
    pub fn add_random_points(&mut self,  num_of_points : usize) {
        self.point_collection = Vec::with_capacity(num_of_points);
        while self.point_collection.len() < num_of_points {
            let candidate = Vec2::new(rand::random_range(0.0..EXTENSION),
                                      rand::random_range(0.0..EXTENSION));

            if self.point_collection.iter().all(|partner| candidate.dist_to(partner) > 2.0 * EXCLUSION_RADIUS) {
                self.point_collection.push(candidate);
            }
        }
    }

    /// Tries to generate a number of random links.
    /// After too many attempts it stops adding links.
    /// 
    /// # Example
    /// ```
    /// use astar_lib::graph_constructor::GraphConstructor;
    /// let mut constructor = GraphConstructor::new();
    /// constructor.add_random_points(1000);
    /// constructor.try_add_links(5000);
    /// ``` 
    pub fn try_add_links(&mut self, num_of_links : usize) {
        self.point_pairing = Vec::with_capacity(num_of_links);
        let mut counter = 0;
        let num_of_points = self.point_collection.len();
        let mut link_collection : Vec<Line> = Vec::with_capacity(num_of_links);
        while(self.point_pairing.len() < num_of_links) && (counter < MAX_ITERATIONS)
        {
            counter += 1;
            let first_ind = rand::random_range(0..num_of_points);
            let second_ind = rand::random_range(0..num_of_points);

            // Check if dummy line.
            if first_ind == second_ind { continue; }

            // Check if line already contained in one form.
            let test_pairing = (first_ind, second_ind);
            let test_paring_inverse = (second_ind, first_ind);
            if self.point_pairing.iter().any(|other| (other == &test_pairing  ) || (other == &test_paring_inverse)) {continue}
          

            let line = Line::new(self.point_collection[first_ind], self.point_collection[second_ind] );
            // Check if line too long.
            if line.length() > MAX_LINE_LENGTH {continue;}
            // Check if line intersects with a different one.
            if link_collection.iter().any(|other_line| other_line.intersects_with(&line)) {continue;}

            link_collection.push(line);
            self.point_pairing.push(test_pairing);
        }
    }

    /// Generates a graph
    /// 
    /// # Panic
    /// Points and links have to be added upfront.
    /// 
    /// # Example
    /// ```
    /// use astar_lib::graph_constructor::GraphConstructor;
    /// let mut constructor = GraphConstructor::new();
    /// constructor.add_random_points(1000);
    /// constructor.try_add_links(5000);
    /// let _graph = constructor.generate_graph();
    /// ```
    pub fn generate_graph(&self) -> NavGraph {

        assert!( self.point_collection.len() > 0 && self.point_pairing.len() > 0, "Graph not initialized.");
        let mut graph = NavGraph::new();

        let point_handle: Vec<usize> = self.point_collection.iter().map(|position| graph.add_node(*position)).collect();
        for (first, second) in &self.point_pairing {
            graph.connect_nodes(point_handle[*first], point_handle[*second]);
        }
        graph
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec_construction_test() {
       let mut constructor = GraphConstructor::new();
        constructor.add_random_points(1000);
        constructor.try_add_links(5000);
        let graph = constructor.generate_graph();
        assert_eq!(graph.get_all_nodes_with_state().count(), 1000);
    }

}