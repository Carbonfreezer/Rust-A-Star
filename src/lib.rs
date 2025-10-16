//! This is the library for the A\*  algorithm applied to two-dimensional navigation graphs.
//! It comes with an relatively extensive example of an open GL app, you can run 
//! with "cargo run -r --example openglapp"


#![no_std]
extern crate alloc;

pub mod a_star;
#[doc(hidden)]
pub mod vector;

