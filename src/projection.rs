use std::fmt::Debug;

use bevy::{math::InvalidDirectionError, prelude::*};

use crate::grounding::is_walkable;

#[derive(Reflect, Debug, PartialEq, Clone, Copy)]
#[reflect(Debug, Clone)]
pub struct Surface {
    pub normal: Dir3,
    pub is_walkable: bool,
}

impl Surface {
    pub(crate) fn new<D>(normal: D, walkable_angle: f32, up_direction: Dir3) -> Self
    where
        D: TryInto<Dir3>,
        <D as TryInto<Dir3>>::Error: Debug,
    {
        let normal = normal.try_into().unwrap();
        Self {
            normal,
            is_walkable: is_walkable(*normal, walkable_angle, *up_direction),
        }
    }

    pub fn obstruction_normal(
        &self,
        current_ground_normal: Option<Dir3>,
        up_direction: Dir3,
    ) -> Result<Dir3, InvalidDirectionError> {
        if !self.is_walkable
            && let Some(ground_normal) = current_ground_normal
        {
            // Calculate obstruction normal perpendicular to both ground normal and the up direction
            let tangent = Dir3::new(ground_normal.cross(*self.normal))?;
            return Dir3::new(tangent.cross(*up_direction));
        }

        Ok(self.normal)
    }

    #[must_use]
    pub fn align_velocity(&self, velocity: Vec3, up_direction: Dir3) -> Vec3 {
        align_with_surface(velocity, *self.normal, *up_direction)
    }

    #[must_use]
    pub fn project_velocity(
        &self,
        velocity: Vec3,
        current_ground_normal: Option<Dir3>,
        up_direction: Dir3,
    ) -> Vec3 {
        // TODO: why not just do this in the method instead?
        let obstruction_normal = self
            .obstruction_normal(current_ground_normal, up_direction)
            .unwrap_or(self.normal);

        project_velocity(
            velocity,
            *obstruction_normal,
            self.is_walkable,
            current_ground_normal,
            up_direction,
        )
    }
}

/// Represents the current state of collision resolution during movement.
#[derive(Reflect, Default, Debug, Clone, Copy)]
#[reflect(Default, Debug, Clone)]
pub enum CollisionState {
    /// Initial state before any collision.
    #[default]
    Initial,
    /// Character has collided with a single plane.
    Plane { previous_surface: Surface },
    /// Character has collided with two planes forming a crease.
    Crease,
    /// Character has reached a corner or complex geometry that prevents further movement.
    Corner,
}

impl CollisionState {
    pub fn reset(&mut self) {
        *self = Self::Initial;
    }

    #[must_use]
    pub fn update(
        &mut self,
        surface: Surface,
        velocity: Vec3,
        previous_velocity: Vec3,
        is_grounded: bool,
        mut project_velocity: impl FnMut(Vec3) -> Vec3,
    ) -> Vec3 {
        // Short-circuit for walkable surfaces - always allow movement
        if surface.is_walkable {
            return project_velocity(velocity);
        }

        match *self {
            // First collision
            CollisionState::Initial => {
                *self = Self::Plane {
                    previous_surface: surface,
                };
                project_velocity(velocity)
            }
            // Second collision
            CollisionState::Plane { previous_surface } => {
                if let Some(crease) = detect_crease(
                    surface,
                    previous_surface,
                    velocity,
                    previous_velocity,
                    is_grounded,
                ) {
                    // If grounded, stop movement at creases
                    if is_grounded {
                        *self = Self::Corner;
                        Vec3::ZERO
                    } else {
                        // Otherwise project along the crease direction
                        *self = Self::Crease;
                        velocity.project_onto(*crease)
                    }
                } else {
                    *self = Self::Plane {
                        previous_surface: surface,
                    };
                    project_velocity(velocity)
                }
            }
            // Third collision, stop movement
            CollisionState::Crease => {
                *self = Self::Corner;
                Vec3::ZERO
            }
            CollisionState::Corner => Vec3::ZERO,
        }
    }
}

pub(crate) fn project_velocity(
    velocity: Vec3,
    obstruction_normal: Vec3,
    is_walkable: bool,
    current_ground_normal: Option<Dir3>,
    up_direction: Dir3,
) -> Vec3 {
    match (current_ground_normal, is_walkable) {
        // Character on ground, moving to walkable surface
        (Some(_), true) => {
            // Align the velocity to the surface while maintaining the horizontal direction
            align_with_surface(velocity, obstruction_normal, *up_direction)
        }
        // Character on ground, moving to non-walkable surface
        (Some(ground_normal), false) => {
            // Calculate the right vector along the intersection of ground and obstruction
            let ground_right = obstruction_normal.cross(*ground_normal).normalize_or_zero();

            // Calculate the up vector along the ground that's perpendicular to obstruction
            let ground_up = ground_right.cross(obstruction_normal).normalize_or_zero();

            // Align with the ground and slide along the obstruction
            align_with_surface(velocity, ground_up, *up_direction).reject_from(obstruction_normal)
        }
        // Character in air, hitting walkable surface
        (None, true) => {
            // Remove the vertical component and align with the surface
            let velocity = velocity.reject_from(*up_direction);
            align_with_surface(velocity, obstruction_normal, *up_direction)
        }
        // Character in air, hitting non-walkable surface
        (None, false) => {
            // Simply slide along the surface
            velocity.reject_from(obstruction_normal)
        }
    }
}

/// Detects if two colliding surfaces form a crease that requires special handling.
pub(crate) fn detect_crease(
    current_surface: Surface,
    previous_surface: Surface,
    current_velocity: Vec3,
    previous_velocity: Vec3,
    is_grounded: bool,
) -> Option<Dir3> {
    // Skip if both surfaces are walkable and character is currently grounded
    if is_grounded && current_surface.is_walkable && previous_surface.is_walkable {
        return None;
    }

    // Skip if normals are nearly parallel
    if current_surface.normal.dot(*previous_surface.normal) > 1.0 - 1e-3 {
        return None;
    }

    // Calculate the direction of the crease
    let mut crease_direction =
        Dir3::new(current_surface.normal.cross(*previous_surface.normal)).unwrap();

    // Project normals onto the plane perpendicular to the crease
    let current_normal_on_crease_plane =
        *Dir3::new(current_surface.normal.reject_from(*crease_direction)).unwrap();
    let previous_normal_on_crease_plane =
        *Dir3::new(previous_surface.normal.reject_from(*crease_direction)).unwrap();

    // Project previous velocity onto the crease plane
    let entering_velocity_on_crease_plane = previous_velocity.reject_from(*crease_direction);

    // Check if the angle between planes indicates a concave corner and if the velocity is going into it
    let dot_planes_on_crease_planes = current_normal_on_crease_plane.dot(*previous_surface.normal);

    if dot_planes_on_crease_planes
        > entering_velocity_on_crease_plane.dot(-current_normal_on_crease_plane) + 1e-3
        || dot_planes_on_crease_planes
            > entering_velocity_on_crease_plane.dot(-previous_normal_on_crease_plane) + 1e-3
    {
        return None;
    }

    // Flip the crease direction to match the direction of the current velocity
    if crease_direction.dot(current_velocity) < 0.0 {
        crease_direction = -crease_direction;
    }

    Some(crease_direction)
}

/// Align the vector with the `normal` plane along the `up` axis.
///
/// The returned vector maintains the same total magnitude as the input vector.
pub(crate) fn align_with_surface(vector: Vec3, normal: Vec3, up: Vec3) -> Vec3 {
    let right = vector.cross(up);
    let forward = normal.cross(right);
    forward.normalize_or_zero() * vector.length()
}
