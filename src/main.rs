use bevy::prelude::*;

use bevy::{app::AppExit, window::PresentMode};

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(0.0, 0.0, 0.0)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Platformer Jump Checker".to_string(),
                present_mode: PresentMode::AutoVsync,
                ..default()
            }),
            ..default()
        }))
        // Startup systems
        .add_systems(Startup, s_init)
        // Update systems
        .add_systems(Update, s_input)
        .run();
}

pub fn s_init(mut commands: Commands) {
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
