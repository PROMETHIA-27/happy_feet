use std::mem;

use avian3d::{prelude::*, sync::PreviousGlobalTransform};
use bevy::prelude::*;

use crate::{KinematicVelocity, OnGroundLeave, ground::Grounding};

/// A kinematic rigidbody that is moved and rotated using `Transform`.
#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
#[require(RigidBody = RigidBody::Kinematic)]
pub struct PhysicsMover;

pub(crate) fn update_physics_mover(
    mut query: Query<
        (
            &PreviousGlobalTransform,
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
        ),
        With<PhysicsMover>,
    >,
) {
    for (prev_transform, mut transform, mut linear_vel, mut angular_vel) in &mut query {
        let prev_transform = prev_transform.compute_transform();

        linear_vel.0 = transform.translation - prev_transform.translation;
        angular_vel.0 =
            transform.rotation.to_scaled_axis() - prev_transform.rotation.to_scaled_axis();

        transform.translation = prev_transform.translation;
        transform.rotation = prev_transform.rotation;
    }
}

pub(crate) fn update_platform_velocity(
    mut characters: Query<(
        &KinematicVelocity,
        &Grounding,
        &mut InheritedVelocity,
        &Transform,
    )>,
    platform_colliders: Query<&ColliderOf>,
    platforms: Query<(
        Option<&LinearVelocity>,
        Option<&AngularVelocity>,
        Option<&ComputedCenterOfMass>,
        &GlobalTransform,
    )>,
    time: Res<Time>,
) -> Result {
    for (velocity, grounding, mut velocity_on_platform, transform) in &mut characters {
        let Some(platform_collider) = grounding.inner_ground else {
            *velocity_on_platform = InheritedVelocity::default();
            continue;
        };

        let platform = platform_colliders
            .get(platform_collider.entity)
            .map(|it| it.body)
            .unwrap_or(platform_collider.entity);

        let (
            platform_linear_velocity,
            platform_angular_velocity,
            platform_center_of_mass,
            platform_transform,
        ) = platforms.get(platform)?;

        let platform_position = platform_transform
            .transform_point(platform_center_of_mass.map(|it| it.0).unwrap_or_default());

        *velocity_on_platform = InheritedVelocity::at_point(
            platform_position,
            platform_linear_velocity.map(|it| it.0).unwrap_or_default(),
            platform_angular_velocity.map(|it| it.0).unwrap_or_default(),
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
