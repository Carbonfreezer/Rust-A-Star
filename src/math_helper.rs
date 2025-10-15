//! This module contains various helper function.

use std::ops::{Add, Sub};

/// Contains a two dimensional vector.
#[derive(Debug, Copy, Clone)]
pub struct Vec2 {
    x: f32,
    y: f32,
}

impl Vec2 {
    /// Creates a new vector, we can add and subtract those.
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// let test = Vec2::new(1.0, 2.0);
    /// ```
    pub fn new(x: f32, y: f32) -> Vec2 {
        Vec2 { x, y }
    }

    /// Returns the position as an vertex array.
    /// Primarily intended for the use of OpenGL
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// let test = Vec2::new(1.0, 2.0);
    /// let vec = test.get_as_array();
    /// ```
    pub fn get_as_array(&self) -> [f32; 2] {
        [self.x, self.y]
    }

    /// Generates an array combination of our vector and a second one.
    /// This is a specific function implemented to draw a line with OpenGL.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// let test_a = Vec2::new(1.0, 2.0);
    /// let test_b = Vec2::new(0.0, 1.0);
    /// let vec = test_a.get_combined_as_array(&test_b);
    /// ```
    pub fn get_combined_as_array(&self, other: &Vec2) -> [f32; 4] {
        [self.x, self.y, other.x, other.y]
    }

    /// Gets the magnitude and a normalized version of this vector in one call.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// let test = Vec2::new(1.0, 2.0);
    /// let (mag, norm) = test.get_mag_normalized();
    /// assert_eq!(mag, test.magnitude(), "They should be the same.")
    /// ```
    pub fn get_mag_normalized(&self) -> (f32, Vec2) {
        let mag = self.magnitude();
        let norm_vec = Vec2::new(self.x / mag, self.y / mag);
        (mag, norm_vec)
    }

    /// Computes the dort product of this vector with another.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// let test_a = Vec2::new(1.0, 0.0);
    /// let test_b = Vec2::new(0.0, 1.0);
    /// let dot = test_a.dot(test_b);
    /// assert_eq!(dot, 0.0, "orthogonal vectors")
    /// ```
    pub fn dot(&self, other: Vec2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// Gets an orthogonal version to this vector.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// let test_a = Vec2::new(1.0, 2.0);
    /// let test_b = test_a.get_orthogonal();
    /// let dot = test_a.dot(test_b);
    /// assert_eq!(dot, 0.0, "orthogonal vectors")
    /// ```
    pub fn get_orthogonal(&self) -> Vec2 {
        Vec2::new(self.y, -self.x)
    }

    /// Gets the magnitude of a vector.
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// let vec = Vec2::new(1.0, 2.0);
    /// let size = vec.magnitude();
    /// ```
    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Computes the distance to another vector.
    /// # Example
    /// ```
    /// use astar_lib::math_helper::Vec2;
    /// let p1 = Vec2::new(1.0, 2.0);
    /// let p2 = Vec2::new(2.0, 2.0);
    /// let d = p1.dist_to(&p2);
    /// ```
    pub fn dist_to(&self, other: &Vec2) -> f32 {
        (*self - *other).magnitude()
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, other: Vec2) -> Vec2 {
        Vec2 {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, other: Vec2) -> Vec2 {
        Vec2 {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

/// Contains a line segment. can be used for intersection calculation.
#[derive(Debug, Clone)]
pub struct Line {
    start: Vec2,
    delta: Vec2,
    magnitude: f32,
    unit_delta: Vec2,
    orthogonal: Vec2,
}

const EPSILON: f32 = 0.00001;

impl Line {
    /// Creates a new line.
    /// # Example
    /// ```
    /// use astar_lib::math_helper::{Vec2, Line};
    /// let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    /// ```
    pub fn new(start: Vec2, end: Vec2) -> Line {
        let delta = end - start;
        let (magnitude, unit_delta) = delta.get_mag_normalized();
        let orthogonal = unit_delta.get_orthogonal();
        Line {
            start,
            delta,
            magnitude,
            unit_delta,
            orthogonal,
        }
    }

    /// Checks if an indicated point is in the voronoi region of the edge and if its distance
    /// is lower than the indicated range.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::{Vec2, Line};
    /// let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    /// let critical = line_a.is_in_critical_range(Vec2::new(0.5, 0.5), 0.001);
    /// assert!(critical, "We should be right on the line.");
    /// ```
    pub fn is_in_critical_range(&self, test_point: Vec2, range: f32) -> bool {
        let rel_to_start = test_point - self.start;

        // First we check if we are in the voronoi region of the edge.
        let rel_dist = rel_to_start.dot(self.unit_delta);
        if !(EPSILON..self.magnitude - EPSILON).contains(&rel_dist) {
            return false;
        }

        let orthogonal_dist = self.orthogonal.dot(rel_to_start).abs();
        orthogonal_dist <= range
    }

    /// Checks if this line intersects with another line.
    ///
    /// # Example
    /// ```
    /// use astar_lib::math_helper::{Vec2, Line};
    /// let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    /// let line_b = Line::new(Vec2::new(0.0, 1.0), Vec2::new(1.0, 0.0));
    /// let intersect = line_a.intersects_with(&line_b);
    /// assert!(intersect);
    /// ```
    pub fn intersects_with(&self, other: &Line) -> bool {
        let start_delta = other.start - self.start;

        let base_det = -self.delta.x * other.delta.y + self.delta.y * other.delta.x;
        let own_det = -start_delta.x * other.delta.y + start_delta.y * other.delta.x;
        let other_det = self.delta.x * start_delta.y - self.delta.y * start_delta.x;

        let my = own_det / base_det;
        let lambda = other_det / base_det;

        (EPSILON..1.0 - EPSILON).contains(&my) && (EPSILON..1.0 - EPSILON).contains(&lambda)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec_test() {
        let vec_a = Vec2::new(0.0, 0.0);
        let vec_b = Vec2::new(1.0, 1.0);
        let dist = vec_a.dist_to(&vec_b);
        assert!((dist - (2.0_f32).sqrt()).abs() < 0.00000000001);
    }

    #[test]
    fn line_test() {
        let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
        let line_b = Line::new(Vec2::new(0.0, 1.0), Vec2::new(1.0, 0.0));
        let line_c = Line::new(Vec2::new(1.0, 0.0), Vec2::new(1.0, 1.0));
        let intersect = line_a.intersects_with(&line_b);
        assert!(intersect);
        let intersect = line_c.intersects_with(&line_b);
        assert!(!intersect);
    }
}
