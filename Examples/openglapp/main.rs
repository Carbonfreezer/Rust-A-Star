//! This is an interactive OpenGL app, that demonstrates the functionality of the library.
//!

pub mod graph_constructor;
pub mod graphics;
pub mod line;

extern crate astar_lib;

/// Simply calls the run_prog of the graphics module.
pub fn main() {
    graphics::run_prog(0.015, 0.04, 0.25, 0.05, 300, 800)
}
