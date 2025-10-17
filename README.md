# A\* lib
This is the library for the A\*  algorithm applied to two-dimensional navigation graphs as often used in games.

# General Usage
The application domain of this library is two-dimensional navigation graphs. Every node in this
graph represents a two-dimensional position. When nodes are connected with an edge, the edge gets
annotated with the Euclidean distance between these two nodes. The library provides functionalities to add nodes
and to add and remove edges. The latter may be helpful in game applications when the world, and therefore the navigation options, change.
The main functionality of the system is to deliver the shortest path between two specified positions.

For debug and visualization purposes, the nodes and edges with their respective state can also be queried from the outside.
Also, a node within a certain distance of an indicated point may be queried, which can be used for picking
applications.

Two-dimensional coordinates are always represented as a **\[f32,2\]** in the interface of the program.


# Sample Code
A simplistic example to use the program looks like this:

```
use astar_lib::a_star::NavGraph;
let mut graph = NavGraph::new();
let p0 = graph.add_node([0.0, 0.0]);
let p1 = graph.add_node([0.5, 0.5]);
let p2 = graph.add_node([1.0, 0.0]);
let p3 = graph.add_node([1.0, 1.0]);
let p4 = graph.add_node([0.1, 0.0]);
graph.connect_nodes(p0, p1).unwrap();
graph.connect_nodes(p1, p2).unwrap();
graph.connect_nodes(p0, p2).unwrap();
graph.connect_nodes(p1, p4).unwrap();
graph.connect_nodes(p4, p3).unwrap();
graph.connect_nodes(p2, p3).unwrap();

let result = graph.search_graph(p0, p3);

if let Some(result) = result {
     for pos in result.iter() {
          println!("{:?}", pos);
} }
``` 


# Examples 
The library comes with the example app *openglapp*.
This example app contains several modules:

1. **line**: This contains the *Line* class in there that helps with the graph construction.
2. **graph_constructor**: This is a helper module that generates random graphs that obey a couple of rules to be pretty.
3. **graphics**: This module does the visualization with OpenGL and the basic interaction.

To start the demo app, use

```
cargo run -r --example openglapp
```

With the left mouse button, you can pick one of the graph nodes. On mouse-over a different node, the system
computes and generates the shortest path if possible, and displays the path and the states of the nodes in the A*
algorithm after completion. By clicking the right mouse button, a new graph gets generated.

We show an example in the following image:

<figure>
    <img src="graph_shot.png" alt="Image of the graph" width="600" height="600">
    <figcaption>Screenshot of the A Star Program</figcaption>
</figure>

# Getting Started with Rust
If you are new to Rust, here is a quick start:

1. Install Rust
2. Build, run, and test the various components.

## Install Rust
For *Linux* and *MacOS* users, open a terminal and enter the following command:
```
curl --proto '=https' --tlsv1.3 https://sh.rustup.rs -sSf | sh
```
For *Windows* users, get to the website
[Windows Installer](https://www.rust-lang.org/tools/install).

In both cases, you will wind up with mainly three programs:
- **rustup**: This is the installer and updater.
- **rustc**: This is the core compiler of the Rust language. You will rarely interface with it directly.
- **cargo**: This program contains the package manager (something like PiPy in Python) and a complete build system.
  This program is the central entry to the Rust world.

## Build, Run, and Test the various components
Once you have installed Rust, clone the directory from the repository, open a terminal, and navigate to the base directory
where the file *Cargo.toml* is contained. From here, you may now run several commands:

- **cargo test**: This builds the program and runs all contained unit tests.
- **cargo doc --open**: Generates and opens the documentation in the browser.
- **cargo run -r --example openglapp** : Start the app.


# License
The program is published under the MIT license as explained in the [license file](LICENSE).