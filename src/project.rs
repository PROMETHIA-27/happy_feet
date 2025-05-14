use bevy::prelude::*;
use itertools::Itertools;

#[derive(Component, Reflect, Debug, Default, Clone)]
#[reflect(Component, Default)]
pub(crate) struct SlidePlanes {
    pub planes: Vec<Dir3>,
} // TODO: smallvec

#[derive(Debug, Clone, Copy)]
pub(crate) enum PlaneType {
    Plane(Dir3),
    Crease { direction: Dir3, tangent: Dir3 },
    Corner,
}

impl PlaneType {
    #[must_use]
    pub fn project_motion(self, motion: Vec3) -> Vec3 {
        match self {
            PlaneType::Plane(normal) => motion.reject_from_normalized(*normal),
            PlaneType::Crease { direction, .. } => motion.project_onto_normalized(*direction),
            PlaneType::Corner => Vec3::ZERO,
        }
    }
}

impl SlidePlanes {
    pub fn clear(&mut self) {
        self.planes.clear();
    }

    /// Insert a new plane if it doesn't exist, returning the resulting [`PlaneType`].
    #[track_caller]
    pub fn push(&mut self, normal: impl TryInto<Dir3>) -> bool {
        // TODO: we only really have to normalize after checking if the normal is unique
        let normal = normal
            .try_into()
            .unwrap_or_else(|_| panic!("normal must not be zero, infinite or NaN"));

        // Make sure the no two normals are the same
        if self.planes.iter().any(|n| n.dot(*normal) > 1.0 - 0.01) {
            return false;
        }

        self.planes.push(normal);

        true
    }

    pub fn pop(&mut self) -> Option<Dir3> {
        self.planes.pop()
    }

    pub fn normals(&self) -> impl Iterator<Item = Dir3> {
        self.planes.iter().copied()
    }

    pub fn creases(&self) -> impl Iterator<Item = Dir3> {
        self.planes
            .iter()
            .array_combinations()
            .filter(|&[a, b]| a.dot(**b) < 0.0)
            .map(|[a, b]| Dir3::new(a.cross(**b)).unwrap())
    }

    pub fn is_corner(&self) -> bool {
        for [a, b, c] in self.planes.iter().array_combinations() {
            if a.dot(**b) < 0.0 && b.dot(**c) < 0.0 && a.dot(**c) < 0.0 {
                return true;
            }
        }

        false
    }

    pub fn project_velocity(&self, mut velocity: Vec3, original_direction: Dir3) -> Vec3 {
        match &self.planes[..] {
            [] => velocity,
            &[plane] => velocity.reject_from(*plane),
            planes => {
                for &plane in planes {
                    velocity = velocity.reject_from(*plane);
                }

                for [a, b] in planes.iter().array_combinations() {
                    if a.dot(**b) < 0.0 {
                        let crease = a.cross(**b);
                        velocity = velocity.project_onto(crease);
                    }
                }

                for [a, b, c] in planes.iter().array_combinations() {
                    if a.dot(**b) < 0.0 && b.dot(**c) < 0.0 && a.dot(**c) < 0.0 {
                        return Vec3::ZERO;
                    }
                }

                if velocity.dot(*original_direction) < 0.0 {
                    velocity = Vec3::ZERO;
                }

                velocity
            }
        }
    }
}

/// Projects a vector on a walkable plane.
///
/// The returned vector will be aligned with the plane `normal` with the horizontal direction unchanged.
///
/// **Panics** if the `normal` is zero, infinite or `NaN`.
#[track_caller]
#[must_use]
pub(crate) fn project_motion_on_ground(motion: Vec3, normal: impl TryInto<Dir3>, up: Dir3) -> Vec3 {
    let normal = normal
        .try_into()
        .unwrap_or_else(|_| panic!("normal must not be zero, infinite or NaN"));

    // Split input vector into vertical and horizontal components
    let mut vertical = motion.project_onto(*up);
    let mut horizontal = motion - vertical;

    // Remove downward velocity
    if vertical.dot(*normal) < 0.0 {
        vertical = Vec3::ZERO;
    }

    let Ok(tangent) = Dir3::new(horizontal.cross(*up)) else {
        return vertical; // No horizontal velocity
    };

    // Calculate the horizontal direction along the slope
    // This gives us a vector that follows the slope but maintains original horizontal intent
    let Ok(horizontal_direction) = Dir3::new(tangent.cross(*normal)) else {
        // Horizontal direction is already perpendicular with the ground normal
        return vertical + horizontal;
    };

    // Project horizontal movement onto the calculated direction
    horizontal = horizontal.project_onto_normalized(*horizontal_direction);

    (vertical + horizontal).normalize_or_zero() * motion.length()
}

/// Projects a vector on a non-walkable plane.
///
/// The returned vector will be aligned with both the `normal` and `up` direction.
///
/// This ensures the character is not able to slide up slopes that are not walkable.
///
/// **Panics** if the `normal` is zero, infinite or `NaN`.
#[track_caller]
#[must_use]
pub(crate) fn project_motion_on_wall(motion: Vec3, normal: impl TryInto<Dir3>, up: Dir3) -> Vec3 {
    let normal = normal
        .try_into()
        .unwrap_or_else(|_| panic!("normal must not be zero, infinite or NaN"));

    // Split input vector into vertical and horizontal components
    let mut vertical = motion.project_onto(*up);
    let mut horizontal = motion - vertical;

    // Project the vertical part on the plane normal
    vertical = vertical.reject_from_normalized(*normal);

    let Ok(tangent) = Dir3::new(normal.cross(*up)) else {
        return vertical + horizontal; // This is not a wall
    };

    // Project horizontal movement along the wall tangent
    horizontal = horizontal.project_onto_normalized(*tangent);

    vertical + horizontal
}
