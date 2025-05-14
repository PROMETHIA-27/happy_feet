use avian3d::{math::PI, parry::shape::TypedShape, prelude::*};
use bevy::{color::palettes::css::*, prelude::*};

enum GroundState {
    None,
    Walkable,
    Steep,
    Unsupported,
}

enum Plane {
    Floor,
    Slope,
    Wall,
    Roof,
}

impl Plane {
    fn new(normal: Dir3, up_direction: Dir3) -> Self {
        let d = normal.dot(*up_direction);

        if d > 0.0 {
            return match d < 1.0 {
                true => Self::Slope,
                false => Self::Floor,
            };
        }

        if d < 0.0 {
            return match d > -1.0 {
                true => Self::Slope,
                false => Self::Roof,
            };
        }

        Self::Wall
    }
}

struct MovementSettings {
    up: Dir3,
    max_slope_angle: f32,
}

fn compute_ground_movement_vector(
    movement: Vec3,
    up_direction: Dir3,
    normal: Option<Dir3>,
    maintain_horizontal_movement: bool,
) -> Vec3 {
    let projected_movement = movement.project_onto_normalized(*up_direction);

    if maintain_horizontal_movement {
        return projected_movement;
    }

    projected_movement.normalize_or_zero() * movement.length()
}
