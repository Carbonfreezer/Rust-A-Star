//! This is essentially the core program containing all the OpenGL graphics.

use crate::a_star::{NavGraph, NodeState};
use crate::graph_constructor::GraphConstructor;
use crate::math_helper::Vec2;
use glume::gl;
use glume::gl::types::*;
use glume::window::{Event, MouseButton};

pub struct InteractionCore {
    screen_extension: (f32, f32),
    cursor_pos: Vec2,
    shader_program: u32,
    translation: i32,
    color: i32,
    line_vbo_vba: (u32, u32),
    circle_vba: u32,
    graph_constructor: GraphConstructor,
    num_of_points: usize,
    num_of_links: usize,
    graph: NavGraph,
    circle_radius: f32,
    node_selected: Option<usize>,
}

const POINTS_IN_CIRCLE: usize = 20;

impl InteractionCore {
    /// Generates the interaction core from several parameters.
    ///
    /// # Parameters
    /// * **circle_radius**: The radius with which we paint circles. Also used for clicking.
    /// * **circle_exclusion_radius**: An outer radius for a point to keep points aparts (poisson disc distribution).
    /// * **max_line_length**: The maximum length an edge in the graph may have.
    /// * **num_of_points**: The number of nodes we have in the graph.
    /// * **num_of_links**: The number of edge we have in the graph-
    ///
    pub fn new(
        circle_radius: f32,
        circle_exclusion_radius: f32,
        max_line_length: f32,
        num_of_points: usize,
        num_of_links: usize,
    ) -> InteractionCore {
        let shader_program = Self::create_shader_program();
        let (translation, color) = Self::get_const_shader(shader_program);
        let line_vbo_vba = Self::create_line_vbo_and_vba();
        let circle_vba = Self::create_circle_vba(circle_radius);
        let mut graph_constructor =
            GraphConstructor::new(1.0, max_line_length, circle_exclusion_radius);
        graph_constructor.add_random_points(num_of_points);
        graph_constructor.add_random_links(num_of_links);
        let graph = graph_constructor.generate_graph();

        InteractionCore {
            screen_extension: (100.0, 100.0),
            cursor_pos: Vec2::new(0.0, 0.0),
            shader_program,
            translation,
            color,
            line_vbo_vba,
            circle_vba,
            graph_constructor,
            num_of_points,
            num_of_links,
            graph,
            circle_radius,
            node_selected: None,
        }
    }

    fn create_line_vbo_and_vba() -> (u32, u32) {
        let vertices: Vec<f32> = vec![0.0, 0.0, 0.0, 0.0];
        let mut vbo: u32 = 0;
        let mut vba: u32 = 0;

        unsafe {
            gl::CreateBuffers(1, &mut vbo);
            gl::CreateVertexArrays(1, &mut vba);

            gl::BindVertexArray(vba);
            gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
            gl::BufferData(
                gl::ARRAY_BUFFER,
                (vertices.len() * size_of::<f32>()) as GLsizeiptr,
                vertices.as_ptr() as *const GLvoid,
                gl::DYNAMIC_DRAW,
            );

            gl::VertexAttribPointer(
                0,
                2,
                gl::FLOAT,
                gl::FALSE,
                (2 * size_of::<f32>()) as GLint,
                std::ptr::null(),
            );
            gl::EnableVertexAttribArray(0);
            gl::BindVertexArray(0);
            gl::BindBuffer(gl::ARRAY_BUFFER, 0);
        }

        (vbo, vba)
    }

    fn create_circle_vba(radius: f32) -> u32 {
        let mut vertices: Vec<f32> = Vec::with_capacity(POINTS_IN_CIRCLE * 2);

        for i in 0..POINTS_IN_CIRCLE {
            vertices.push(
                radius * (i as f32 * 2.0 * std::f32::consts::PI / POINTS_IN_CIRCLE as f32).cos(),
            );
            vertices.push(
                radius * (i as f32 * 2.0 * std::f32::consts::PI / POINTS_IN_CIRCLE as f32).sin(),
            );
        }

        let mut vbo: u32 = 0;
        let mut vba: u32 = 0;

        unsafe {
            gl::CreateBuffers(1, &mut vbo);
            gl::CreateVertexArrays(1, &mut vba);

            gl::BindVertexArray(vba);
            gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
            gl::BufferData(
                gl::ARRAY_BUFFER,
                (vertices.len() * size_of::<f32>()) as GLsizeiptr,
                vertices.as_ptr() as *const GLvoid,
                gl::STATIC_DRAW,
            );

            gl::VertexAttribPointer(
                0,
                2,
                gl::FLOAT,
                gl::FALSE,
                (2 * size_of::<f32>()) as GLint,
                std::ptr::null(),
            );
            gl::EnableVertexAttribArray(0);
            gl::BindVertexArray(0);
            gl::BindBuffer(gl::ARRAY_BUFFER, 0);
        }

        vba
    }

    fn get_const_shader(program: u32) -> (i32, i32) {
        let color_str = std::ffi::CString::new("PaintColor").unwrap();
        let translation_str = std::ffi::CString::new("translation").unwrap();
        let translation;
        let color;
        unsafe {
            translation = gl::GetUniformLocation(program, translation_str.as_ptr());
            color = gl::GetUniformLocation(program, color_str.as_ptr());
        }

        (translation, color)
    }

    fn compile_shader(source: &str, shader_type: u32) -> u32 {
        let shader = unsafe { gl::CreateShader(shader_type) };
        let c_str = std::ffi::CString::new(source).unwrap();
        unsafe {
            gl::ShaderSource(shader, 1, &c_str.as_ptr(), std::ptr::null());
            gl::CompileShader(shader);
        }

        shader
    }

    fn create_shader_program() -> u32 {
        let v_code = r#"
            #version 330
            uniform vec2 translation;
            layout(location = 0) in vec2 position;
            void main()
            {
	            gl_Position = vec4(position + translation,  0.0,  1.0);
            }
            "#;

        let f_code = r#"
            #version 330
            uniform vec3 PaintColor;
            out vec4 color;
            void main()
            {
                color = vec4(PaintColor, 1.0);
            }
            "#;

        let v_shader = Self::compile_shader(v_code, gl::VERTEX_SHADER);
        let f_shader = Self::compile_shader(f_code, gl::FRAGMENT_SHADER);

        unsafe {
            let program = gl::CreateProgram();
            gl::AttachShader(program, v_shader);
            gl::AttachShader(program, f_shader);
            gl::LinkProgram(program);
            gl::DetachShader(program, v_shader);
            gl::DetachShader(program, f_shader);
            gl::DeleteShader(v_shader);
            gl::DeleteShader(f_shader);

            program
        }
    }

    fn draw_circle(&self, center: &Vec2, color: &[f32]) {
        let color_ptr = color.as_ptr();
        let center_array = center.get_as_array();
        let position_ptr = center_array.as_ptr();

        unsafe {
            // Upload variables.
            gl::Uniform3fv(self.color, 1, color_ptr);
            gl::Uniform2fv(self.translation, 1, position_ptr);
            gl::DrawArrays(gl::TRIANGLE_FAN, 0, POINTS_IN_CIRCLE as i32);
        }
    }

    fn draw_line(&self, start: &Vec2, end: &Vec2, color: &[f32]) {
        let vertices: Vec<f32> = start
            .get_as_array()
            .iter()
            .chain(end.get_as_array().iter())
            .copied()
            .collect();
        let color_ptr = color.as_ptr();
        let zero_vec = [0.0_f32, 0.0_f32];
        let position_ptr = zero_vec.as_ptr();

        unsafe {
            gl::Uniform3fv(self.color, 1, color_ptr);
            gl::Uniform2fv(self.translation, 1, position_ptr);
            gl::BufferData(
                gl::ARRAY_BUFFER,
                (vertices.len() * size_of::<f32>()) as GLsizeiptr,
                vertices.as_ptr() as *const GLvoid,
                gl::DYNAMIC_DRAW,
            );
            gl::DrawArrays(gl::LINES, 0, 2);
        }
    }

    fn get_color(state: &NodeState) -> [f32; 3] {
        match state {
            NodeState::Clear => [0.1_f32, 0.5_f32, 0.1_f32],
            NodeState::Visited => [1.0_f32, 1.0_f32, 0.0_f32],
            NodeState::Closed => [1.0_f32, 0.0_f32, 0.0_f32],
            NodeState::Solution => [0.0_f32, 0.0_f32, 1.0_f32],
        }
    }

    /// Gets invoked to render everything new. First paints the lines and then the nodes.
    pub fn redraw(&self) {
        unsafe {
            gl::UseProgram(self.shader_program);
            gl::BindVertexArray(self.line_vbo_vba.1);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.line_vbo_vba.0);
        }
        for (start, end, solution) in self.graph.get_all_links_with_solution_hint() {
            let color_state = if solution {
                NodeState::Solution
            } else {
                NodeState::Clear
            };
            self.draw_line(start, end, &Self::get_color(&color_state))
        }
        unsafe {
            gl::BindVertexArray(self.circle_vba);
        }
        for (position, state) in self.graph.get_all_nodes_with_state() {
            self.draw_circle(position, &Self::get_color(state));
        }
        unsafe {
            gl::BindVertexArray(0);
            gl::BindBuffer(gl::ARRAY_BUFFER, 0);
            gl::UseProgram(0);
        }
    }

    /// Sets the window extension from the outside. This is needed to get the cursor position
    /// in clip space.
    pub fn set_window_extension(&mut self, width: u32, height: u32) {
        self.screen_extension = (width as f32, height as f32);
    }

    /// Function gets called when the mouse cursor has moved. Stores the position and eventually
    /// updates the graph search calculation-
    pub fn set_cursor_pos(&mut self, (x, y): (f32, f32)) {
        self.cursor_pos = Vec2::new(
            2.0_f32 * x / self.screen_extension.0 - 1.0_f32,
            1.0 - 2.0_f32 * y / self.screen_extension.1,
        );

        // Here we analyze if we have a pick node.
        if let Some(start) = self.node_selected
            && let Some(destination) = self
                .graph
                .find_nearest_node_with_radius(&self.cursor_pos, self.circle_radius)
        {
            self.graph.search_graph(start, destination);
        }
    }

    /// Gets called from the outside on right mouse button to regenerate a new graph.
    pub fn generate_graph(&mut self) {
        self.node_selected = None;
        self.graph_constructor.add_random_points(self.num_of_points);
        self.graph_constructor.add_random_links(self.num_of_links);
        self.graph = self.graph_constructor.generate_graph();
    }

    /// Gets called from the outside on left mouse button to select a new start point.
    pub fn pick_node(&mut self) {
        if let Some(hit_node) = self
            .graph
            .find_nearest_node_with_radius(&self.cursor_pos, self.circle_radius)
        {
            self.node_selected = Some(hit_node);
        }
    }
}

/// This is the main entrance to the test program that starts the OpenGL application
///
///
/// # Parameters
/// * **circle_radius**: The radius with which we paint circles. Also used for clicking.
/// * **circle_exclusion_radius**: An outer radius for a point to keep points aparts (poisson disc distribution).
/// * **max_line_length**: The maximum length an edge in the graph may have.
/// * **num_of_points**: The number of nodes we have in the graph.
/// * **num_of_links**: The number of edge we have in the graph-
/// # Example
/// ```no_run
///  use astar_lib::graphics;
///  graphics::run_prog(0.015, 0.04, 0.2, 300, 800)
/// ```
pub fn run_prog(
    circle_radius: f32,
    circle_exclusion_radius: f32,
    max_line_length: f32,
    num_of_points: usize,
    num_of_links: usize,
) {
    assert!(
        circle_radius < circle_exclusion_radius,
        "The inner radius has to be smaller."
    );

    // initial configuration for the window
    let window_config = glume::window::WindowConfiguration {
        title: "A star".to_string(),
        size: (800, 800),
        gl_version: (3, 3),
    };

    let window = window_config.build_window();

    // after the window is created, we can call OpenGL functions, not before
    unsafe {
        gl::Enable(gl::DEBUG_OUTPUT);
    }

    let mut core = InteractionCore::new(
        circle_radius,
        circle_exclusion_radius,
        max_line_length,
        num_of_points,
        num_of_links,
    );

    window.run(move |wc, event| {
        match event {
            Event::Resized(width, height) => {
                core.set_window_extension(width, height);
                unsafe {
                    gl::Viewport(0, 0, width as i32, height as i32);
                }
            }

            Event::RedrawRequested => {
                unsafe {
                    gl::ClearColor(0.0, 0.0, 0.0, 1.0);
                    gl::Clear(gl::COLOR_BUFFER_BIT);
                    core.redraw();
                };
            }

            Event::CursorMoved(x, y) => {
                core.set_cursor_pos((x, y));
                wc.request_redraw();
            }

            Event::KeyPressed(key) => {
                use glume::window::VirtualKeyCode as Vk;
                if key == Vk::Escape {
                    wc.close()
                }
            }

            Event::MouseButtonPressed(button) => {
                wc.request_redraw();
                match button {
                    MouseButton::Left => {
                        core.pick_node();
                    }
                    MouseButton::Right => core.generate_graph(),
                    _ => {}
                }
            }

            _ => {}
        }
        Ok(())
    })
}
