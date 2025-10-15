//! This is a simple module that only provides an entry point to get a program out of the library.

extern crate astar_lib;
use astar_lib::graphics;

/// Simply calls the run_prog of the graphics module.
pub fn main() {
    graphics::run_prog(0.015, 0.04, 0.25, 0.05, 300, 800)
}
