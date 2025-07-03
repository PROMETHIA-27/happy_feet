use std::mem;

use avian3d::{prelude::*, sync::PreviousGlobalTransform};
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

use crate::{
    character::KinematicVelocity,
    grounding::{Grounding, OnGroundLeave},
};

pub struct MovingPlatformPlugin {
    pub schedule: Interned<dyn ScheduleLabel>,
}

impl Default for MovingPlatformPlugin {
    fn default() -> Self {
        Self::new(FixedPostUpdate.intern())
    }
}

impl MovingPlatformPlugin {
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Plugin for MovingPlatformPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            self.schedule,
            (
                update_physics_mover,
                update_inherited_velocity,
                move_with_platform,
            )
                .before(PhysicsSet::StepSimulation)
                .chain(),
        );

        app.add_observer(apply_inherited_velocity_on_ground_leave);
    }
}

/// A kinematic rigid body that is moved and rotated using `Transform`.
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
    time: Res<Time>,
) {
    for (prev_transform, mut transform, mut linear_vel, mut angular_vel) in &mut query {
        let prev_transform = prev_transform.compute_transform();

        linear_vel.0 = (transform.translation - prev_transform.translation) / time.delta_secs();
        angular_vel.0 = (transform.rotation.to_scaled_axis()
            - prev_transform.rotation.to_scaled_axis())
            / time.delta_secs();

        transform.translation = prev_transform.translation;
        transform.rotation = prev_transform.rotation;
    }
}

pub(crate) fn update_inherited_velocity(
    mut characters: Query<(
        &KinematicVelocity,
        &Grounding,
        &mut InheritedVelocity,
        &Position,
    )>,
    colliders: Query<&ColliderOf>,
    rigid_bodies: Query<(
        &LinearVelocity,
        &AngularVelocity,
        &ComputedCenterOfMass,
        &GlobalTransform,
    )>,
    time: Res<Time>,
) -> Result {
    for (velocity, grounding, mut inherited_velocity, position) in &mut characters {
        let Some(collider) = grounding.inner_ground else {
            inherited_velocity.0 = Vec3::ZERO;
            continue;
        };

        let rigid_body = colliders
            .get(collider.entity)
            .map_or(collider.entity, |it| it.body);

        let Ok((
            platform_linear_velocity,
            platform_angular_velocity,
            platform_center_of_mass,
            platform_transform,
        )) = rigid_bodies.get(rigid_body)
        else {
            inherited_velocity.0 = Vec3::ZERO;
            continue;
        };

        let platform_position = platform_transform.transform_point(platform_center_of_mass.0);

        *inherited_velocity = InheritedVelocity::at_point(
            platform_position,
            platform_linear_velocity.0,
            platform_angular_velocity.0,
            position.0,
            velocity.0,
            time.delta_secs(),
        );
    }

    Ok(())
}

pub(crate) fn move_with_platform(
    mut query: Query<(&mut Position, &InheritedVelocity)>,
    time: Res<Time>,
) {
    for (mut position, inherited_velocity) in &mut query {
        position.0 += inherited_velocity.0 * time.delta_secs();
    }
}

pub(crate) fn apply_inherited_velocity_on_ground_leave(
    trigger: Trigger<OnGroundLeave>,
    mut query: Query<(&mut KinematicVelocity, &mut InheritedVelocity)>,
) {
    let Ok((mut velocity, mut platform_velocity)) = query.get_mut(trigger.target()) else {
        return;
    };

    velocity.0 += mem::take(&mut platform_velocity.0);
}

/// The velocity of the ground a character is standing on.
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
        // Calculate an initial estimate of platform velocity at the current position
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
