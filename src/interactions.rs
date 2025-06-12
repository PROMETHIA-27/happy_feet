use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    Character, CollideAndSlideFilter, KinematicVelocity, Surface,
    ground::{Ground, Grounding, GroundingConfig},
    sweep::{CollideAndSlideConfig, sweep},
};

/// Push dynamic bodies before character movement and physics simulation.
pub(crate) fn physics_interactions(
    spatial_query: SpatialQuery,
    mut characters: Query<
        (
            &Character,
            &CollideAndSlideConfig,
            &mut KinematicVelocity,
            Option<(&mut Grounding, &GroundingConfig)>,
            &Transform,
            &Collider,
            &ComputedMass,
            &CollideAndSlideFilter,
        ),
        Without<Sensor>,
    >,
    mut bodies: Query<(
        &mut LinearVelocity,
        &mut AngularVelocity,
        &ComputedMass,
        &ComputedAngularInertia,
        &GlobalTransform,
        &ComputedCenterOfMass,
        &RigidBody,
    )>,
    time: Res<Time>,
) {
    for (
        character,
        config,
        mut character_velocity,
        grounding,
        character_transform,
        character_collider,
        character_mass,
        filter,
    ) in &mut characters
    {
        let Ok((sweep_direction, target_sweep_distance)) =
            Dir3::new_and_length(character_velocity.0 * time.delta_secs())
        else {
            continue;
        };

        let Some(hit) = sweep(
            character_collider,
            character_transform.translation,
            character_transform.rotation,
            sweep_direction,
            target_sweep_distance,
            config.skin_width,
            &spatial_query,
            &filter.0,
            false,
        ) else {
            continue;
        };

        let Ok((
            mut linear_velocity,
            mut angular_velocity,
            mass,
            inertia,
            transform,
            center_of_mass,
            rb,
        )) = bodies.get_mut(hit.entity)
        else {
            continue;
        };

        if !rb.is_dynamic() {
            continue;
        }

        // Don't push the body the character is standing on
        if let Some((mut grounding, grounding_config)) = grounding {
            let surface = Surface::new(hit.normal, grounding_config.max_angle, character.up);

            // Project velocity and set ground
            if surface.is_walkable {
                character_velocity.0 = surface.project_velocity(
                    character_velocity.0,
                    grounding.normal(),
                    character.up,
                );

                grounding.inner_ground = Some(Ground {
                    entity: hit.entity,
                    normal: surface.normal,
                });
            }

            if grounding.entity() == Some(hit.entity) {
                continue;
            }
        }

        // We have to limit the impulse applied to bodies so that small bodies don't glitch out of existence

        let depth = 1.0 - hit.distance / target_sweep_distance;

        let Ok(impulse_direction) = Dir3::new(-hit.normal) else {
            continue;
        };

        let incoming_speed = character_velocity.0.dot(*impulse_direction);

        if incoming_speed <= 0.0 {
            continue;
        }

        character_velocity.0 -= impulse_direction * incoming_speed * depth;

        let current_linear_speed = linear_velocity.dot(*impulse_direction);

        let linear_force = incoming_speed * depth * character_mass.value() * time.delta_secs();
        let linear_acceleration = f32::min(
            incoming_speed - current_linear_speed,
            linear_force * mass.inverse(),
        );

        linear_velocity.0 += impulse_direction * linear_acceleration;

        // Angular push

        let center_of_mass = transform.transform_point(center_of_mass.0);
        let contact_point = hit.point;
        let contact_offset = contact_point - center_of_mass;

        let torque = contact_offset.cross(impulse_direction * linear_force);

        // Get current angular velocity in the torque direction
        let Ok(torque_axis) = Dir3::new(torque) else {
            continue;
        };
        let current_angular_speed = angular_velocity.0.dot(*torque_axis);

        const MAX_TORQUE: f32 = 100.0;

        let angular_acceleration = f32::min(
            f32::max(0.0, incoming_speed - current_angular_speed),
            f32::min(
                MAX_TORQUE * time.delta_secs(),
                (inertia.inverse() * torque).length(),
            ),
        );

        // Apply limited angular change
        angular_velocity.0 += torque_axis * angular_acceleration;
    }
}

fn apply_impulse_on_point(
    impulse: Vec3,
    point: Vec3,
    linear_velocity: &mut Vec3,
    angular_velocity: &mut Vec3,
    inverse_mass: f32,
    inverse_inertia: Mat3,
    center_of_mass: Vec3,
) {
    *linear_velocity += inverse_mass * impulse;

    let torque = (point - center_of_mass).cross(impulse);
    *angular_velocity += inverse_inertia * torque;
}
