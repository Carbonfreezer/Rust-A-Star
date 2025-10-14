//! This module contains various helper function.

use std::ops::{Add, Mul, Sub};

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
        Vec2 { x: self.x + other.x, y: self.y + other.y }
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, other: Vec2) -> Vec2 {
        Vec2 { x: self.x - other.x, y: self.y - other.y }
    }
}

impl Mul<Vec2> for f32 {
    type Output = Vec2;

    fn mul(self, rhs: Vec2) -> Vec2 {
        Vec2 { x: self * rhs.x, y: self * rhs.y }
    }
}



/// Contains a line segment. can be used for intersection calculation.
#[derive(Debug,  Clone)]
pub struct Line {
    start: Vec2,
    end: Vec2,
}


impl Line {
    /// Creates a new line.
    /// # Example
    /// ```
    /// use astar_lib::math_helper::{Vec2, Line};
    /// let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    /// ```
    pub fn new(start: Vec2, end: Vec2) -> Line {
        Line { start, end }
    }
    
    
    /// Computes the length of a line.
    /// # Example
    /// ```
    /// use astar_lib::math_helper::{Vec2, Line};
    /// let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    /// let length = line_a.length();
    /// ``` 
    pub fn length(&self) -> f32 {
        (self.end - self.start).magnitude()
    }

    /// Gets the start and end point of the line.
    /// # Example
    /// ```
    /// use astar_lib::math_helper::{Vec2, Line};
    /// let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    /// let (start, end) = line_a.get_start_end();
    /// ``` 
    pub fn get_start_end(&self) -> (&Vec2, &Vec2) {
        (&self.start, &self.end)
    }
    
    /// Gets a shortened version of the line by removing the shortening distance from
    /// the start and the end.
    ///
    /// # Panic
    /// If we remove more from the line than the length of the line.
    /// 
    /// # Example
    /// ```
    /// use astar_lib::math_helper::{Vec2, Line};
    /// let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    /// let new_line = line_a.get_shortened_version(0.1);
    /// ``` 
    pub fn get_shortened_version(&self, shorting_distance: f32 ) -> Line {
        let delta = self.end - self.start; 
        let scaling_relation = shorting_distance / delta.magnitude();
        assert!(scaling_relation <  0.5, "Shoring distance is too large");
        
        let start = self.start + scaling_relation * delta;
        let end = self.end - scaling_relation * delta;
        
        Line::new(start, end)
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
        let own_delta = self.end - self.start;
        let other_delta = other.end - other.start;

        let start_delta = other.start - self.start;

        let base_det = - own_delta.x * other_delta.y + own_delta.y * other_delta.x;
        let own_det = - start_delta.x * other_delta.y + start_delta.y * other_delta.x;
        let other_det = own_delta.x * start_delta.y - own_delta.y * start_delta.x;

        let my = own_det / base_det;
        let lambda = other_det / base_det;

        (my >= 0.0001) && (my <= 0.9999) && (lambda >= 0.0001 && lambda <= 0.9999)
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
    
    #[test]
    fn shortening_test() {
        let line_a = Line::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
        let new_line = line_a.get_shortened_version(0.1);
        
        println!("{:?}", new_line);
    }
}