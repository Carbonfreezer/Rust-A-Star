//! Helper module to generate an interesting graph.
//! Which means it should have a certain number of nodes and edges.
//! The nodes should have a certain minimum distance to each other.
//! The edges should not intersect and there should be a minimum distance
//! of a node to an edge. 

use super::math_helper::{Line, Vec2};
use crate::a_star::NavGraph;
use rand::seq::IteratorRandom;

/// The maximum number of iterations we make per attempt for link generation
const MAX_ITERATIONS: usize = 100000;

pub struct GraphConstructor {
    point_collection: Vec<Vec2>,
    point_pairing: Vec<(usize, usize)>,
    extension: f32,
    max_line_length: f32,
    exclusion_distance: f32,
}

impl GraphConstructor {
    /// Generates a new graph constructor
    ///
    /// # Parameters
    /// * **extension:** The graph will have coordinates ranging from -extension..extension
    /// * **max_line_length:** The maximum length and edge can have.
    /// * **exclusion_radius:** The outer radius of each node to keep distance.
    ///
    /// # Example
    /// ```
    /// use astar_lib::graph_constructor::GraphConstructor;
    /// let mut constructor = GraphConstructor::new(1.0, 0.3, 0.02);
    /// ```
    pub fn new(extension: f32, max_line_length: f32, exclusion_radius: f32) -> GraphConstructor {
        GraphConstructor {
            point_collection: vec![],
            point_pairing: vec![],
            extension,
            max_line_length,
            exclusion_distance: 2.0 * exclusion_radius,
        }
    }

    /// Fills the point array with random points.
    ///
    /// # Example
    /// ```
    /// use astar_lib::graph_constructor::GraphConstructor;
    /// let mut constructor = GraphConstructor::new(1.0, 0.3, 0.02);
    /// constructor.add_random_points(1000);
    /// ```
    pub fn add_random_points(&mut self, num_of_points: usize) {
        self.point_collection = Vec::with_capacity(num_of_points);
        let mut counter = 0;
        while (self.point_collection.len() < num_of_points) && (counter < MAX_ITERATIONS) {
            counter += 1;
            let candidate = Vec2::new(
                rand::random_range(-self.extension..self.extension),
                rand::random_range(-self.extension..self.extension),
            );

            if self
                .point_collection
                .iter()
                .all(|partner| candidate.dist_to(partner) > self.exclusion_distance)
            {
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
    /// let mut constructor = GraphConstructor::new(1.0, 0.3, 0.02);
    /// constructor.add_random_points(1000);
    /// constructor.add_random_links(5000);
    /// ```
    pub fn add_random_links(&mut self, num_of_links: usize) {
        self.point_pairing = Vec::with_capacity(num_of_links);
        let mut counter = 0;
        let num_of_points = self.point_collection.len();
        let mut link_collection: Vec<Line> = Vec::with_capacity(num_of_links);
        while (self.point_pairing.len() < num_of_links) && (counter < MAX_ITERATIONS) {
            counter += 1;
            let first_ind = rand::random_range(0..num_of_points);
            let first_pos = self.point_collection[first_ind];

            // Now we filter for other points that are in max len range.
            let partner_index = self
                .point_collection
                .iter()
                .enumerate()
                .filter(|(index, position)| {
                    (*index != first_ind)
                        && (**position - first_pos).magnitude() < self.max_line_length
                })
                .map(|(index, _)| index)
                .choose(&mut rand::rng());

            // In this case we have not found a node, that is not ourself and is close enough.
            if partner_index.is_none() {
                continue;
            }
            let second_ind = partner_index.unwrap();

            // Check if line already contained in one form.
            let test_pairing = (first_ind, second_ind);
            let test_paring_inverse = (second_ind, first_ind);
            if self
                .point_pairing
                .iter()
                .any(|other| (other == &test_pairing) || (other == &test_paring_inverse))
            {
                continue;
            }
            // Check if line too long.
            let line = Line::new(
                self.point_collection[first_ind],
                self.point_collection[second_ind],
            );

            // Check if line intersects with a different one.
            if link_collection
                .iter()
                .any(|other_line| other_line.intersects_with(&line))
            {
                continue;
            }

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
    /// let mut constructor = GraphConstructor::new(1.0, 0.3, 0.02);
    /// constructor.add_random_points(1000);
    /// constructor.add_random_links(5000);
    /// let _graph = constructor.generate_graph();
    /// ```
    pub fn generate_graph(&mut self) -> NavGraph {
        assert!(
            !self.point_collection.is_empty() && !self.point_pairing.is_empty(),
            "Graph not initialized."
        );
        let mut graph = NavGraph::new();

        let point_handle: Vec<usize> = self
            .point_collection
            .iter()
            .map(|position| graph.add_node(*position))
            .collect();
        for (first, second) in &self.point_pairing {
            graph.connect_nodes(point_handle[*first], point_handle[*second]);
        }
        self.point_collection.clear();
        self.point_pairing.clear();
        graph
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec_construction_test() {
        let mut constructor = GraphConstructor::new(1.0, 0.3, 0.02);
        constructor.add_random_points(1000);
        constructor.add_random_links(5000);
        constructor.generate_graph();
    }
}
