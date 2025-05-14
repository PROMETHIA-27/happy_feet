use std::collections::VecDeque;

use avian3d::prelude::*;
use bevy::{color::palettes::css::*, prelude::*};

use crate::{Character, CharacterMovement, MoveInput, is_walkable};

pub(crate) fn plugin(app: &mut App) {
    app.insert_gizmo_config(
        CharacterGizmos,
        GizmoConfig {
            line: GizmoLineConfig {
                width: 8.0,
                joints: GizmoLineJoint::Bevel,
                ..Default::default()
            },
            depth_bias: -0.5,
            ..Default::default()
        },
    );

    app.add_systems(
        FixedPostUpdate,
        (draw_input_acceleration, draw_debug_motion),
    );
}

#[derive(GizmoConfigGroup, Reflect, Default)]
struct CharacterGizmos;

fn draw_input_acceleration(
    mut gizmos: Gizmos<CharacterGizmos>,
    query: Query<(
        &Transform,
        &Character,
        &CharacterMovement,
        &Collider,
        &MoveInput,
    )>,
) {
    for (transform, character, movement, collider, move_input) in &query {
        let half_height = collider
            .aabb(Vec3::ZERO, transform.rotation)
            .size()
            .dot(character.up * 0.5);

        // let vertical_velocity = character.velocity.project_onto_normalized(*character.up);
        // let horizontal_verlocity = character.velocity - vertical_velocity;

        // if let Ok((direction, speed)) = Dir3::new_and_length(horizontal_verlocity) {
        //     let speed_len = speed / movement.target_speed;
        //     let clamped_speed_len = speed_len.min(1.0);
        //     let rest_speed_len = speed_len - clamped_speed_len;

        //     let origin = transform.translation - character.up * half_height;

        //     gizmos.line(origin, origin + direction * clamped_speed_len, YELLOW);

        //     gizmos.line_gradient(
        //         origin + *direction,
        //         origin + direction * (1.0 + rest_speed_len),
        //         YELLOW,
        //         RED,
        //     );
        // }

        if let Ok(direction) = Dir3::new(move_input.previous()) {
            let speed_len = character.velocity.dot(*direction) / movement.target_speed;
            let accel_len = 1.0 - speed_len.clamp(0.0, 1.0);

            let origin = transform.translation - character.up * half_height;
            let right = character.up.cross(*direction);

            let color = BLACK
                .mix(&VIOLET, accel_len)
                // .mix(&YELLOW, accel_len.powi(2))
                .mix(&RED, accel_len.powi(4));

            let arrow_head = 0.1;

            gizmos.linestrip(
                [
                    origin + *direction - right * arrow_head,
                    origin + direction * (1.0 + arrow_head),
                    origin + *direction + right * arrow_head,
                ],
                color,
            );
        }
    }
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
pub struct DebugMode;

#[derive(Reflect, Debug, Clone, Copy)]
pub(crate) struct DebugHit {
    pub point: Vec3,
    pub normal: Vec3,
}

#[derive(Reflect, Debug, Clone, Copy)]
pub(crate) struct DebugPoint {
    pub translation: Vec3,
    pub velocity: Vec3,
    pub hit: Option<DebugHit>,
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
pub struct DebugMotion {
    pub(crate) duration: f32,
    pub(crate) points: VecDeque<(f32, DebugPoint)>,
}

impl Default for DebugMotion {
    fn default() -> Self {
        Self::new(32)
    }
}

impl DebugMotion {
    pub fn new(capacity: usize) -> Self {
        Self {
            duration: 0.0,
            points: VecDeque::with_capacity(capacity),
        }
    }

    pub(crate) fn push(&mut self, duration: f32, point: DebugPoint) {
        self.duration += duration;
        self.points.push_back((duration, point));

        if self.points.capacity() < self.points.len() + 1 {
            self.duration = self.points.iter().map(|(d, _)| d).sum();
            self.points.pop_front();
        }
    }

    pub(crate) fn duration_at(&self, index: usize) -> f32 {
        self.points.range(..=index).map(|(d, _)| d).sum()
    }

    pub(crate) fn clear(&mut self) {
        self.duration = 0.0;
        self.points.clear()
    }
}

fn draw_debug_motion(
    mut gizmos: Gizmos<CharacterGizmos>,
    mut query: Query<(
        &Character,
        &CharacterMovement,
        &Collider,
        &Transform,
        &mut DebugMotion,
        Has<DebugMode>,
    )>,
) {
    for (character, movement, collider, transform, mut debug_motion, debug_mode) in &mut query {
        let line_color = |t, velocity: Vec3| {
            let vertical = velocity.project_onto_normalized(*character.up);
            let horizontal = velocity - vertical;

            let target_speed_sq = movement.target_speed * movement.target_speed;

            let h = horizontal.length_squared() / target_speed_sq;
            let v = vertical.length_squared() / target_speed_sq;

            let color = VIOLET.mix(&LIGHT_BLUE, h).mix(&CRIMSON, v);

            color.with_alpha(t)
        };

        // lines
        let mut points = Vec::with_capacity(debug_motion.points.len());

        for i in 0..debug_motion.points.len() {
            let (_, a) = debug_motion.points[i];

            points.push((
                a.translation,
                line_color(
                    debug_motion.duration_at(i) / debug_motion.duration,
                    a.velocity,
                ),
            ));
        }

        // Draw last line to character if debug mode is disabled
        if !debug_mode {
            points.push((transform.translation, line_color(1.0, character.velocity)));
        }

        gizmos.linestrip_gradient(points);

        // hits
        let mut dbg_point = |debug: DebugPoint| {
            let radius = 0.2;

            let color = [LIGHT_GREEN, CRIMSON];

            let (point, normal, color) = match debug.hit {
                Some(hit) => {
                    let point = hit.point + hit.normal * character.skin_width;
                    let normal = hit.normal;

                    let color = match is_walkable(normal, *character.up, character.walkable_angle) {
                        true => color[0],
                        false => color[1],
                    };

                    gizmos.line_gradient(point, point + normal * 0.1, WHITE, WHITE.with_alpha(0.0));

                    if let Ok((direction, speed)) = Dir3::new_and_length(debug.velocity) {
                        gizmos.line_gradient(
                            point + direction * radius,
                            point + direction * (radius + speed / movement.target_speed / 2.0),
                            color,
                            color.with_alpha(0.0),
                        );
                    }

                    (point, normal, color)
                }
                None => {
                    let point =
                        debug.translation + character.feet_position(collider, transform.rotation);

                    let color = color[1];

                    gizmos.line_gradient(
                        point,
                        point - character.up * 0.5,
                        color,
                        color.with_alpha(0.0),
                    );

                    (point, *character.up, color)
                }
            };

            let forward = Dir3::new(normal.cross(*character.up)).unwrap_or(Dir3::NEG_Z);

            let transform = Transform::from_translation(point).looking_to(normal, forward);

            gizmos.circle(
                Isometry3d::new(transform.translation, transform.rotation),
                radius,
                color,
            );
        };

        if debug_mode {
            // Draw hit points
            for i in 0..debug_motion.points.len() {
                let (_, debug) = debug_motion.points[i];
                dbg_point(debug);
            }

            debug_motion.clear();
        } else {
            // Only draw last point at character if debug mode is disabled
            dbg_point(DebugPoint {
                translation: transform.translation,
                velocity: character.velocity,
                hit: character.ground.map(|ground| DebugHit {
                    point: transform.translation
                        + character.feet_position(collider, transform.rotation),
                    normal: *ground.normal,
                }),
            });
        }
    }
}
