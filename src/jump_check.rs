use bevy::{
    app::{App, Plugin, Update},
    ecs::system::Res,
    gizmos::gizmos::Gizmos,
    math::Vec2,
    render::color::Color,
};

use crate::{
    level::Level,
    pathfinding::{Pathfinding, PathfindingGraphNode},
    utils::line_intersect,
    GRAVITY_STRENGTH,
};

pub const JUMP_V_MAX: f32 = 8.0;

pub struct JumpCheckPlugin;

impl Plugin for JumpCheckPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, s_jump_check);
    }
}

pub fn s_jump_check(level: Res<Level>, pathfinding: Res<Pathfinding>, mut gizmos: Gizmos) {
    if pathfinding.start_graph_node.is_none() || pathfinding.goal_graph_node.is_none() {
        return;
    }

    let start_node = &pathfinding.start_graph_node.clone().unwrap();
    let goal_node = &pathfinding.goal_graph_node.clone().unwrap();

    let delta_p = goal_node.position - start_node.position;
    let acceleration = Vec2::new(0.0, -GRAVITY_STRENGTH);
    let v_max = JUMP_V_MAX;
    let b1 = delta_p.dot(acceleration) + v_max * v_max;
    let discriminant = b1 * b1 - acceleration.dot(acceleration) * delta_p.dot(delta_p);

    let mut jump_possible = discriminant >= 0.0;

    let t_low_energy = (4.0 * delta_p.dot(delta_p) / acceleration.dot(acceleration))
        .sqrt()
        .sqrt();
    let launch_velocity = delta_p / t_low_energy - acceleration * t_low_energy / 2.0;
    let timestep = t_low_energy / 10 as f32;

    if jump_possible {
        'polygon: for polygon_index in 0..level.polygons.len() {
            let polygon = &level.polygons[polygon_index];
            'line: for line_index in 1..polygon.points.len() {
                let start_node_on_line = start_node.polygon_index == polygon_index
                    && start_node.line_indicies.contains(&(line_index - 1));
                let goal_node_on_line = goal_node.polygon_index == polygon_index
                    && goal_node.line_indicies.contains(&(line_index - 1));

                if start_node_on_line || goal_node_on_line {
                    continue 'line;
                }

                let line_start = polygon.points[line_index - 1];
                let line_end = polygon.points[line_index];

                let mut prev_pos = start_node.position;

                for i in 1..10 {
                    let t = timestep * i as f32;
                    let pos =
                        start_node.position + launch_velocity * t + acceleration * t * t / 2.0;

                    let intersection = line_intersect(prev_pos, pos, line_start, line_end);

                    if let Some(intersection) = intersection {
                        gizmos.circle_2d(intersection, 5.0, Color::RED);

                        jump_possible = false;
                        break 'polygon;
                    }
                    prev_pos = pos;
                }
                let intersection =
                    line_intersect(prev_pos, goal_node.position, line_start, line_end);

                if let Some(intersection) = intersection {
                    gizmos.circle_2d(intersection, 5.0, Color::RED);

                    jump_possible = false;
                    break 'polygon;
                }
            }
        }
    }

    draw_jump_arc(
        start_node.clone(),
        goal_node.clone(),
        launch_velocity,
        acceleration,
        timestep,
        &mut gizmos,
        if jump_possible {
            Color::GREEN
        } else {
            Color::RED
        }
        .with_a(0.2),
        10.0,
    );
}

pub fn draw_jump_arc(
    start_node: PathfindingGraphNode,
    goal_node: PathfindingGraphNode,
    launch_velocity: Vec2,
    acceleration: Vec2,
    timestep: f32,
    gizmos: &mut Gizmos,
    color: Color,
    radius: f32,
) {
    let start_pos = start_node.position;
    gizmos.circle_2d(start_pos, radius, color);

    let goal_pos = goal_node.position;
    gizmos.circle_2d(goal_pos, radius, color);

    let mut prev_pos = start_pos;

    for i in 1..10 {
        let t = timestep * i as f32;
        let pos = start_pos + launch_velocity * t + acceleration * t * t / 2.0;
        // gizmos.line_2d(prev_pos, pos, color);

        let line_dir = (pos - prev_pos).normalize();

        let line_normal = Vec2::new(-line_dir.y, line_dir.x);

        let line_beginning_offset_1 = prev_pos + line_normal * radius;
        let line_beginning_offset_2 = prev_pos - line_normal * radius;
        let line_end_offset_1 = pos + line_normal * radius;
        let line_end_offset_2 = pos - line_normal * radius;

        // gizmos.line_2d(line_beginning_offset_1, line_beginning_offset_2, color);
        // gizmos.line_2d(line_end_offset_1, line_end_offset_2, color);
        gizmos.line_2d(line_beginning_offset_1, line_end_offset_1, color);
        gizmos.line_2d(line_beginning_offset_2, line_end_offset_2, color);

        prev_pos = pos;
    }
    // gizmos.line_2d(prev_pos, goal_pos, color);

    let line_dir = (goal_pos - prev_pos).normalize();

    let line_normal = Vec2::new(-line_dir.y, line_dir.x);

    let line_beginning_offset_1 = prev_pos + line_normal * radius;
    let line_beginning_offset_2 = prev_pos - line_normal * radius;
    let line_end_offset_1 = goal_pos + line_normal * radius;
    let line_end_offset_2 = goal_pos - line_normal * radius;

    // gizmos.line_2d(line_beginning_offset_1, line_beginning_offset_2, color);
    // gizmos.line_2d(line_end_offset_1, line_end_offset_2, color);

    gizmos.line_2d(line_beginning_offset_1, line_end_offset_1, color);
    gizmos.line_2d(line_beginning_offset_2, line_end_offset_2, color);
}
