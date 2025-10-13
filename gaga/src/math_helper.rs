// !! This module contains various helper function.

use std::ops::{Add, Sub};

/// Contains a two dimensional vector.
#[derive(Debug, Copy, Clone)]
pub struct Vec2 {
    
    x: f32,
    y: f32,
}

impl Vec2 {
    /// Creates a new vector
    /// #Example
    /// ```
    /// let test = Vec2::new(1.0, 2.0)
    /// ```
    pub fn new(x: f32, y: f32) -> Vec2 {
        
        Vec2 { x, y }
    }

    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    
    pub fn dist_to(&self, other: &Vec2) -> f32 {
        (*self - *other).magnitude()
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, other: Vec2) -> Vec2 {
        Vec2 { x: self.x + other.x, y: self.y + other.y }
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, other: Vec2) -> Vec2 {
        Vec2 { x: self.x - other.x, y: self.y - other.y }
    }
}