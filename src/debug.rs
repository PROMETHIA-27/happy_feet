use avian3d::{parry::shape::TypedShape, prelude::*};
use bevy::{color::palettes::css::*, prelude::*};

use crate::{
    character::KinematicVelocity,
    collide_and_slide::CollideAndSlideConfig,
    grounding::{Grounding, GroundingConfig},
    movement::{CharacterMovement, MoveInput, feet_position},
};

pub struct DebugPlugin;

impl Plugin for DebugPlugin {
    fn build(&self, app: &mut App) {
        app.insert_gizmo_config(
            CharacterGizmos,
            GizmoConfig {
                line: GizmoLineConfig {
                    width: 6.0,
                    joints: GizmoLineJoint::Bevel,
                    ..Default::default()
                },
                depth_bias: -0.1,
                ..Default::default()
            },
        );

        app.add_systems(
            FixedPostUpdate,
            (draw_input_arrow, draw_debug_grounding).after(PhysicsSet::Sync),
        );
    }
}

#[derive(GizmoConfigGroup, Reflect, Default)]
pub(crate) struct CharacterGizmos;

#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component, Debug, Clone)]
pub struct DebugInput;

#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component, Debug, Clone)]
pub struct DebugGrounding;

fn draw_input_arrow(
    mut gizmos: Gizmos<CharacterGizmos>,
    query: Query<
        (
            &Position,
            &Rotation,
            &KinematicVelocity,
            &CharacterMovement,
            &Collider,
            &MoveInput,
            &GroundingConfig,
        ),
        With<DebugInput>,
    >,
) {
    for (position, rotation, velocity, movement, collider, move_input, grounding_config) in &query {
        let half_height = collider
            .aabb(Vec3::ZERO, rotation.0)
            .size()
            .dot(grounding_config.up_direction * 0.5);

        if let Ok(direction) = Dir3::new(move_input.previous()) {
            let speed_len = velocity.0.dot(*direction) / movement.target_speed;
            let accel_len = 1.0 - speed_len.clamp(0.0, 1.0);

            let origin = position.0 - grounding_config.up_direction * half_height;
            let right = grounding_config.up_direction.cross(*direction);

            let color = BLACK.mix(&VIOLET, accel_len).mix(&RED, accel_len.powi(4));

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

fn draw_debug_grounding(
    mut gizmos: Gizmos<CharacterGizmos>,
    mut query: Query<
        (
            &CollideAndSlideConfig,
            &KinematicVelocity,
            &Grounding,
            &GroundingConfig,
            &CharacterMovement,
            &Collider,
            &Position,
            &Rotation,
        ),
        With<DebugGrounding>,
    >,
) {
    for (config, velocity, grounding, grounding_config, movement, collider, position, rotation) in
        &mut query
    {
        let feet_position = feet_position(
            collider,
            Isometry3d::new(position.0, rotation.0),
            grounding_config.up_direction,
            config.skin_width,
        );

        let radius = match collider.shape().as_typed_shape() {
            TypedShape::Capsule(capsule) => capsule.radius,
            TypedShape::Cylinder(cylinder) => cylinder.radius,
            TypedShape::Ball(ball) => ball.radius,
            shape => {
                warn_once!("Unsupported collider shape: {shape:?}");
                0.2
            }
        };

        let color = match grounding.is_grounded() {
            true => LIGHT_GREEN,
            false => CRIMSON,
        };

        let up = match grounding.ground() {
            Some(ground) => ground.normal,
            None => grounding_config.up_direction,
        };

        let feet_rotation = Transform::from_rotation(rotation.0)
            .looking_to(up, grounding_config.up_direction)
            .rotation;

        gizmos.circle(Isometry3d::new(feet_position, feet_rotation), radius, color);

        if let Ok((direction, length)) = Dir3::new_and_length(velocity.0.reject_from(*up)) {
            gizmos.line_gradient(
                feet_position + direction * radius,
                feet_position + direction * (radius + length / movement.target_speed * 0.5),
                color,
                color.with_alpha(0.0),
            );
        }

        if let Some(ground) = grounding.ground() {
            gizmos.line_gradient(
                feet_position,
                feet_position + ground.normal * 0.2,
                color,
                color.with_alpha(0.0),
            );
        }
    }
}
