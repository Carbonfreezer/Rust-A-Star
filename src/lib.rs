//! This is the library for the A\*  algorithm applied to two-dimensional navigation graphs.
//! It comes with a relatively extensive example of an open GL app, you can run
//! with "cargo run -r --example openglapp"
//!
//! # General usage.
//! The application domain of this library is two-dimensional navigation graphs. Every node in this
//! graph represents a two-dimensional position. When nodes are connected with an edge, the edge gets
//! annotated with the Euclidean distance of these two nodes. The library provides functionalities to add nodes
//! and to add and remove edges. The latter may be helpful in game applications when the world, and therefore the navigation options, change.
//! The main functionality of the system is to deliver the shortest path between two specified positions.
//!
//! For debug and visualization purposes, the nodes and edges with their respective state can also be queried from the outside.
//! Also, a node within a certain distance of an indicated point may be queried, which can be used for picking
//! applications.
//!
//! Two-dimensional coordinates are always represented as a **\[f32,2\]** in the interface of the program.
//!
//! # Contents
//! The *a_star* module contains the following enums and structures:
//!
//! * **NavGraph**: This is the central class that provides the complete functionality.
//! * **ConnectionError**: This enum represents the various errors that may happen during establishing and releasing connections.
//! * **NodeState**: You will not need this structure unless you plan to make a debug visualization of the NavGraph  
//!   

pub mod a_star;
#[doc(hidden)]
pub mod vector;
