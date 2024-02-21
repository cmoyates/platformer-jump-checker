mod jump_check;
mod level;
mod pathfinding;
mod utils;

use bevy::prelude::*;

use bevy::window::PrimaryWindow;
use bevy::{app::AppExit, window::PresentMode};
use jump_check::JumpCheckPlugin;
use level::{generate_level_polygons, Level};
use pathfinding::{init_pathfinding_graph, Pathfinding, PathfindingPlugin};

pub const GRAVITY_STRENGTH: f32 = 0.5;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(0.0, 0.0, 0.0)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Platformer Jump Checker".to_string(),
                present_mode: PresentMode::AutoVsync,
                focused: true,
                ..default()
            }),
            ..default()
        }))
        .add_plugins(PathfindingPlugin)
        .add_plugins(JumpCheckPlugin)
        // Startup systems
        .add_systems(Startup, s_init)
        // Update systems
        .add_systems(Update, s_input)
        .add_systems(Update, s_render)
        .run();
}

pub fn s_init(mut commands: Commands, pathfinding: ResMut<Pathfinding>) {
    let grid_size = 32.0;

    let (level_polygons, size, half_size) = generate_level_polygons(grid_size);

    let level = Level {
        polygons: level_polygons,
        grid_size,
        size,
        half_size,
    };

    init_pathfinding_graph(&level, pathfinding);

    commands.insert_resource(level);

    commands.spawn(Camera2dBundle::default());
}

pub fn s_input(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut exit: EventWriter<AppExit>,
    mouse_input: Res<ButtonInput<MouseButton>>,
    q_windows: Query<&Window, With<PrimaryWindow>>,
    mut pathfinding: ResMut<Pathfinding>,
) {
    // Escape to exit (if not WASM)
    #[cfg(not(target_arch = "wasm32"))]
    if keyboard_input.just_pressed(KeyCode::Escape) {
        exit.send(AppExit);
    }

    // Select start node with left click
    if mouse_input.just_pressed(MouseButton::Left) {
        let window_size = q_windows.single().resolution.clone();
        if let Some(position) = q_windows.single().cursor_position() {
            let mut mouse_pos_world =
                position - Vec2::new(window_size.width() / 2.0, window_size.height() / 2.0);
            mouse_pos_world.y *= -1.0;

            for node_index in 0..pathfinding.nodes.len() {
                let node = &pathfinding.nodes[node_index];

                if (mouse_pos_world - node.position).length_squared() < (3.5_f32).powi(2) {
                    pathfinding.start_graph_node = Some(node.clone());
                }
            }
        }
    }
    // Select goal node with right click
    if mouse_input.just_pressed(MouseButton::Right) {
        let window_size = q_windows.single().resolution.clone();
        if let Some(position) = q_windows.single().cursor_position() {
            let mut mouse_pos_world =
                position - Vec2::new(window_size.width() / 2.0, window_size.height() / 2.0);
            mouse_pos_world.y *= -1.0;

            for node_index in 0..pathfinding.nodes.len() {
                let node = &pathfinding.nodes[node_index];

                if (mouse_pos_world - node.position).length_squared() < (7.5_f32).powi(2) {
                    pathfinding.goal_graph_node = Some(node.clone());
                }
            }
        }
    }
}

pub fn s_render(mut gizmos: Gizmos, level: Res<Level>, pathfinding: Res<Pathfinding>) {
    // Draw the level polygons
    for polygon in &level.polygons {
        gizmos.linestrip_2d(
            polygon.points.iter().cloned().collect::<Vec<Vec2>>(),
            polygon.color.with_a(0.1),
        );
    }

    // Draw the pathfinding nodes
    for node in &pathfinding.nodes {
        gizmos.circle_2d(node.position, 2.5, Color::WHITE);
    }

    // Draw a larger circle for the start and end nodes
    if let Some(start_graph_node) = &pathfinding.start_graph_node {
        gizmos.circle_2d(start_graph_node.position, 2.5, Color::GREEN);
    }
    if let Some(goal_graph_node) = &pathfinding.goal_graph_node {
        gizmos.circle_2d(goal_graph_node.position, 2.5, Color::YELLOW);
    }
}
