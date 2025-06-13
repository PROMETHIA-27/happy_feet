use std::mem;

use avian3d::{prelude::*, sync::PreviousGlobalTransform};
use bevy::prelude::*;

use crate::{KinematicVelocity, OnGroundLeave, ground::Grounding};

/// A kinematic rigidbody that is moved using `Transform::translation`.
#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
#[require(RigidBody = RigidBody::Kinematic)]
pub struct PhysicsMover;

pub(crate) fn update_physics_mover(
    mut query: Query<
        (
            // &mut PreviousTransform,
            &mut PreviousGlobalTransform,
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
        ),
        With<PhysicsMover>,
    >,
) {
    for (mut prev_trans, mut transform, mut linear_vel, mut angular_vel) in &mut query {
        let delta_pos = transform.translation - prev_trans.translation();
        transform.translation = prev_trans.translation();
        linear_vel.0 = delta_pos;

        // TODO: angular velocity
    }
}

pub(crate) fn update_platform_velocity(
    mut characters: Query<(
        &KinematicVelocity,
        &Grounding,
        &mut InheritedVelocity,
        &Transform,
    )>,
    platforms: Query<(
        &LinearVelocity,
        &AngularVelocity,
        &GlobalTransform,
        &ComputedCenterOfMass,
    )>,
    time: Res<Time>,
) -> Result {
    for (velocity, grounding, mut velocity_on_platform, transform) in &mut characters {
        let Some(platform) = grounding.inner_ground else {
            *velocity_on_platform = InheritedVelocity::default();
            continue;
        };

        let (
            platform_linear_velocity,
            platform_angular_velocity,
            platform_transform,
            platform_center_of_mass,
        ) = platforms.get(platform.entity)?;

        let platform_position = platform_transform.transform_point(platform_center_of_mass.0);

        *velocity_on_platform = InheritedVelocity::at_point(
            platform_position,
            platform_linear_velocity.0,
            platform_angular_velocity.0,
            transform.translation,
            velocity.0,
            time.delta_secs(),
        );
    }

    Ok(())
}

pub(crate) fn inherit_platform_velocity(
    trigger: Trigger<OnGroundLeave>,
    mut query: Query<(&mut KinematicVelocity, &mut InheritedVelocity)>,
) {
    let Ok((mut velocity, mut platform_velocity)) = query.get_mut(trigger.target()) else {
        return;
    };
    velocity.0 += mem::take(&mut platform_velocity.0);
}

pub(crate) fn move_with_platform(
    mut query: Query<(&mut Transform, &InheritedVelocity)>,
    time: Res<Time>,
) {
    for (mut transform, inherited_velocity) in &mut query {
        transform.translation += inherited_velocity.0 * time.delta_secs();
    }
}

/// The velocity of the ground a character is standing ong.
#[derive(Component, Reflect, Default, Debug)]
#[reflect(Component)]
pub struct InheritedVelocity(pub Vec3);

impl InheritedVelocity {
    pub fn at_point(
        translation: Vec3,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
        point: Vec3,
        relative_velocity: Vec3,
        delta: f32,
    ) -> Self {
        // Calculate an initial estimate of platform velocity at current position
        let initial_velocity =
            inherited_velocity_at_point(translation, linear_velocity, angular_velocity, point);

        // Calculate midpoint position (where we'll be halfway through the timestep)
        let midpoint_position = point + (relative_velocity + initial_velocity) * delta / 2.0;

        // Calculate the final platform velocity
        let velocity = inherited_velocity_at_point(
            translation,
            linear_velocity,
            angular_velocity,
            midpoint_position,
        );

        Self(velocity)
    }
}

pub(crate) fn inherited_velocity_at_point(
    translation: Vec3,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    point: Vec3,
) -> Vec3 {
    let mut inherited_velocity = linear_velocity;

    // Angular velocity component
    if angular_velocity.length_squared() > 0.0 {
        // Vector from platform center to character
        let radius_vector = point - translation;

        // Tangential velocity = angular_velocity × radius_vector
        inherited_velocity += angular_velocity.cross(radius_vector);
    };

    inherited_velocity
}
