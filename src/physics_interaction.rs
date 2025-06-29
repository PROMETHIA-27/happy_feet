use avian3d::prelude::*;
use bevy::prelude::*;

use crate::character::{CharacterHit, KinematicVelocity};

pub struct PhysicsInteractionPlugin;

impl Plugin for PhysicsInteractionPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(physics_interaction_on_hit);
    }
}

fn physics_interaction_on_hit(
    trigger: Trigger<CharacterHit>,
    mut commands: Commands,
    mut bodies: Query<(
        &RigidBody,
        &Transform,
        &Position,
        &Rotation,
        &mut LinearVelocity,
        &mut AngularVelocity,
        &ComputedMass,
        &ComputedAngularInertia,
        &ComputedCenterOfMass,
    )>,
    mut characters: Query<(&mut KinematicVelocity, &ComputedMass)>,
    time: Res<Time>,
) {
    let (mut character_velocity, character_mass) = characters.get_mut(trigger.target()).unwrap();

    let Ok((
        rb,
        transform,
        position,
        rotation,
        mut linear_vel,
        mut angular_vel,
        mass,
        inertia,
        center_of_mass,
    )) = bodies.get_mut(trigger.hit.entity)
    else {
        return;
    };

    if !rb.is_dynamic() {
        return;
    }

    let impulse = trigger.velocity.project_onto(*trigger.hit.surface.normal);

    let Ok((direction, acceleration)) = Dir3::new_and_length(impulse) else {
        return;
    };

    let transform = Transform {
        translation: position.0,
        rotation: rotation.0,
        scale: transform.scale,
    };

    apply_acceleration_on_point(
        *direction,
        acceleration * character_mass.value(),
        acceleration,
        trigger.hit.point,
        &mut linear_vel,
        &mut angular_vel,
        mass.inverse(),
        inertia.inverse(),
        transform.transform_point(center_of_mass.0),
        time.delta_secs(),
    );

    commands.entity(trigger.hit.entity).remove::<Sleeping>();

    character_velocity.0 += trigger.hit.direction * trigger.hit.distance;
}

fn apply_acceleration_on_point(
    direction: Vec3,
    max_acceleration: f32,
    target_speed: f32,
    point: Vec3,
    linear_velocity: &mut Vec3,
    angular_velocity: &mut Vec3,
    inverse_mass: f32,
    inverse_inertia: Mat3,
    center_of_mass: Vec3,
    delta: f32,
) {
    // Linear push

    let linear_speed = linear_velocity.dot(direction);

    if linear_speed < target_speed {
        // Clamp to avoid acceleration past the target speed.
        let linear_accel = f32::min(
            target_speed - linear_speed,
            max_acceleration * delta * inverse_mass,
        );

        *linear_velocity += direction * linear_accel;
    }

    // Angular push

    let contact_offset = point - center_of_mass;

    let torque = contact_offset.cross(direction * max_acceleration * delta);

    let Ok(torque_axis) = Dir3::new(torque) else {
        return;
    };

    // Get current angular velocity in the torque direction
    let angular_speed = angular_velocity.dot(*torque_axis);

    if angular_speed < target_speed {
        let angular_accel = f32::min(
            target_speed - angular_speed,
            (inverse_inertia * torque).length(),
        );

        *angular_velocity += torque_axis * angular_accel;
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
