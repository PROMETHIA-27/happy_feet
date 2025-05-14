use avian3d::prelude::*;
use bevy::{color::palettes::css::*, prelude::*};

use crate::{Character, CharacterMovement, MoveInput, ground};

pub(crate) fn plugin(app: &mut App) {
    app.insert_gizmo_config(
        CharacterGizmoConfig,
        GizmoConfig {
            line: GizmoLineConfig {
                width: 24.0,
                perspective: true,
                joints: GizmoLineJoint::Miter,
                ..Default::default()
            },
            depth_bias: -0.5,
            ..Default::default()
        },
    );

    app.add_systems(FixedPostUpdate, debug_character_movement);
}

#[derive(GizmoConfigGroup, Reflect, Default)]
struct CharacterGizmoConfig;

fn debug_character_movement(
    mut gizmos: Gizmos<CharacterGizmoConfig>,
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

        let vertical_velocity = character.velocity.project_onto_normalized(*character.up);
        let horizontal_verlocity = character.velocity - vertical_velocity;

        if let Ok((direction, speed)) = Dir3::new_and_length(horizontal_verlocity) {
            let speed_len = speed / movement.target_speed;
            let clamped_speed_len = speed_len.min(1.0);
            let rest_speed_len = speed_len - clamped_speed_len;

            let origin = transform.translation - character.up * half_height;

            gizmos.line(origin, origin + direction * clamped_speed_len, YELLOW);

            gizmos.line_gradient(
                origin + *direction,
                origin + direction * (1.0 + rest_speed_len),
                YELLOW,
                RED,
            );
        }

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
