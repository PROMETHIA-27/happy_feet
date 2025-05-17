use bevy::prelude::*;

use crate::is_walkable;

#[derive(Debug, Clone, Copy)]
pub(crate) struct SurfacePlane {
    /// The normal of the plane.
    pub surface_normal: Vec3,
    /// The normal used to slide the character.
    pub obstruction_normal: Vec3,
    /// Whether or not the 'surface_normal' is walkable.
    pub is_walkable: bool,
}

impl SurfacePlane {
    pub fn new(
        surface_normal: Vec3,
        is_walkable: bool,
        ground_normal: Option<Vec3>,
        up: Dir3,
    ) -> Self {
        let mut obstruction_normal = surface_normal;

        if !is_walkable {
            if let Some(ground_normal) = ground_normal {
                // Calculate obstruction normal perpendicular to both ground normal and the up direction
                let side_vector = ground_normal.cross(surface_normal).normalize_or_zero();
                obstruction_normal = side_vector.cross(*up).normalize_or_zero();
            }
        }

        Self {
            is_walkable,
            surface_normal,
            obstruction_normal,
        }
    }

    pub fn project_velocity(
        &self,
        mut velocity: Vec3,
        ground_normal: Option<Vec3>,
        up: Dir3,
    ) -> Vec3 {
        if let Some(ground_normal) = ground_normal {
            if self.is_walkable {
                // Align the velocity to the surface without affecting horizontal direction
                velocity = align_with_surface(velocity, self.obstruction_normal, *up);
            } else {
                let ground_right = self
                    .obstruction_normal
                    .cross(ground_normal)
                    .normalize_or_zero();

                let ground_up = ground_right
                    .cross(self.obstruction_normal)
                    .normalize_or_zero();

                velocity = align_with_surface(velocity, ground_up, *up)
                    .reject_from(self.obstruction_normal);
            }
        } else {
            if self.is_walkable {
                velocity = velocity.reject_from(*up);
                velocity = align_with_surface(velocity, self.obstruction_normal, *up);
            } else {
                velocity = velocity.reject_from(self.obstruction_normal);
            }
        }

        velocity
    }
}

#[derive(Default, Debug)]
pub(crate) enum CollisionState {
    #[default]
    Initial,
    Plane(SurfacePlane),
    Crease,
    Stop,
}

impl CollisionState {
    #[must_use]
    pub fn project_velocity(
        &mut self,
        plane: SurfacePlane,
        velocity: Vec3,
        previous_velocity: Vec3,
        ground_normal: Option<Vec3>,
        mut project_velocity: impl FnMut(Vec3) -> Vec3,
    ) -> Vec3 {
        if plane.is_walkable {
            return project_velocity(velocity);
        }

        match *self {
            CollisionState::Initial => {
                *self = Self::Plane(plane);
                project_velocity(velocity)
            }
            CollisionState::Plane(previous_plane) => {
                if let Some(crease) = get_crease(
                    &plane,
                    &previous_plane,
                    velocity,
                    previous_velocity,
                    ground_normal.is_some(),
                ) {
                    if ground_normal.is_some() {
                        *self = Self::Stop;
                        Vec3::ZERO
                    } else {
                        *self = Self::Crease;
                        velocity.project_onto(crease)
                    }
                } else {
                    *self = Self::Plane(plane);
                    project_velocity(velocity)
                }
            }
            CollisionState::Crease => {
                *self = Self::Stop;
                Vec3::ZERO
            }
            CollisionState::Stop => Vec3::ZERO,
        }
    }
}

fn get_crease(
    current_plane: &SurfacePlane,
    previous_plane: &SurfacePlane,
    current_velocity: Vec3,
    previous_velocity: Vec3,
    is_grounded: bool,
) -> Option<Vec3> {
    if is_grounded && current_plane.is_walkable && previous_plane.is_walkable {
        return None;
    }

    if current_plane
        .surface_normal
        .dot(previous_plane.surface_normal)
        > 1.0 - 1e-4
    {
        return None;
    }

    let mut crease_direction = current_plane
        .surface_normal
        .cross(previous_plane.surface_normal)
        .normalize_or_zero();

    let current_normal_on_crease_plane = current_plane
        .surface_normal
        .reject_from(crease_direction)
        .normalize_or_zero();

    let previous_normal_on_crease_plane = previous_plane
        .surface_normal
        .reject_from(crease_direction)
        .normalize_or_zero();

    let entering_velocity_on_crease_plane = previous_velocity.reject_from(crease_direction);

    let dot_planes_on_crease_planes =
        current_normal_on_crease_plane.dot(previous_plane.surface_normal);

    if dot_planes_on_crease_planes
        <= (entering_velocity_on_crease_plane.dot(-current_normal_on_crease_plane) + 1e-3)
        && dot_planes_on_crease_planes
            <= (entering_velocity_on_crease_plane.dot(-previous_normal_on_crease_plane) + 1e-3)
    {
        // Flip the crease direction to match the direction of the current velocity
        if crease_direction.dot(current_velocity) < 0.0 {
            crease_direction = -crease_direction;
        }

        return Some(crease_direction);
    }

    None
}

/// Align the vector with the `normal` plane along the `up` axis.
pub(crate) fn align_with_surface(vector: Vec3, normal: Vec3, up: Vec3) -> Vec3 {
    let right = vector.cross(up);
    let forward = normal.cross(right).normalize_or_zero();
    forward * vector.length()
}
