//! Provides basic functionality for two-dimensional vectors.

use std::ops::{Add, Sub};

/// Contains a two dimensional vector.
#[derive(Debug, Copy, Clone)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    /// Creates a new vector, we can add and subtract those.
    /// # Example
    /// ```
    /// use astar_lib::vector::Vec2;
    /// let test = Vec2::new(1.0, 2.0);
    /// ```
    pub fn new(x: f32, y: f32) -> Vec2 {
        Vec2 { x, y }
    }
    
    
    /// Gets the magnitude and a normalized version of this vector in one call.
    ///
    /// # Example
    /// ```
    /// use astar_lib::vector::Vec2;
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
    /// use astar_lib::vector::Vec2;
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
    /// use astar_lib::vector::Vec2;
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
    /// use astar_lib::vector::Vec2;
    /// let vec = Vec2::new(1.0, 2.0);
    /// let size = vec.magnitude();
    /// ```
    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Computes the distance to another vector.
    /// # Example
    /// ```
    /// use astar_lib::vector::Vec2;
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

impl From<[f32;2]> for Vec2 {
    fn from(v: [f32;2]) -> Vec2
    {
        Vec2{x: v[0], y: v[1]}
    }
}

impl From<Vec2> for [f32;2] {
    fn from(v: Vec2) -> [f32; 2]
    {
        [v.x, v.y]
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


}
