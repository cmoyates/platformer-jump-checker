mod level;
mod pathfinding;
mod utils;

use bevy::prelude::*;

use bevy::{app::AppExit, window::PresentMode};
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
    mut clear_color: ResMut<ClearColor>,
) {
    // Escape to exit (if not WASM)
    #[cfg(not(target_arch = "wasm32"))]
    if keyboard_input.just_pressed(KeyCode::Escape) {
        exit.send(AppExit);
    }

    if mouse_input.just_pressed(MouseButton::Left) {
        println!("Left mouse button pressed");
        clear_color.0 = Color::rgb(0.0, 0.0, 1.0);
    }
}

pub fn s_render(mut gizmos: Gizmos, level: Res<Level>, pathfinding: Res<Pathfinding>) {
    // Draw the level polygons
    for polygon in &level.polygons {
        gizmos.linestrip_2d(
            polygon.points.iter().cloned().collect::<Vec<Vec2>>(),
            polygon.color,
        );
    }

    // Draw the pathfinding nodes
    for node in &pathfinding.nodes {
        gizmos.circle_2d(node.position, 2.5, Color::WHITE);
    }
}
