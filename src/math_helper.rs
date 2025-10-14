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
    pub fn get_as_array(&self) -> Vec<f32> {
        vec![self.x, self.y]
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
}

impl Line {
    /// Creates a new line.
    /// # Example
    /// ```
    /// use astar_lib::math_helper::{Vec2, Line};
    /// let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    /// ```
    pub fn new(start: Vec2, end: Vec2) -> Line {
        Line {
            start,
            delta: end - start,
        }
    }

    /// Computes the length of a line.
    /// # Example
    /// ```
    /// use astar_lib::math_helper::{Vec2, Line};
    /// let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    /// let length = line_a.length();
    /// ```
    pub fn length(&self) -> f32 {
        self.delta.magnitude()
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

        (0.00001..=0.99999).contains(&my) && (0.00001..=0.99999).contains(&lambda)
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
