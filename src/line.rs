//! Provides line functionality, that is used in the creation of graphs.


use crate::vector::Vec2;

const EPSILON: f32 = 0.00001;


/// Contains a line segment. can be used for intersection calculation.
#[derive(Debug, Clone)]
pub struct Line {
    start: Vec2,
    delta: Vec2,
    magnitude: f32,
    unit_delta: Vec2,
    orthogonal: Vec2,
}


impl Line {
    /// Creates a new line.
    /// # Example
    /// ```
    /// use astar_lib::vector::Vec2;
    /// use astar_lib::line::Line;
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
    /// use astar_lib::vector::Vec2;
    /// use astar_lib::line::Line;
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
    /// use astar_lib::vector::Vec2;
    /// use astar_lib::line::Line;
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

