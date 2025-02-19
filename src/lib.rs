use std::mem;

use avian3d::{math::Scalar, prelude::*};
use bevy::prelude::*;

pub const HALF_PI: f32 = 1.618033988749894848204586834365638118_f32;

#[derive(Clone, Copy)]
enum SlopePlane {
    Floor,
    Wall,
    Roof,
}

#[derive(Clone, Copy)]
struct Slope {
    normal: Dir3,
    angle: f32,
    plane: SlopePlane,
}

impl Slope {
    fn new(up: Dir3, normal: Dir3, max_angle: f32) -> Self {
        let angle = f32::acos(normal.dot(*up));
        let plane = if angle > HALF_PI {
            SlopePlane::Roof
        } else if angle > max_angle {
            SlopePlane::Wall
        } else {
            SlopePlane::Floor
        };
        Self {
            normal,
            angle,
            plane,
        }
    }
}

enum SkinWidth {
    Absolute(f32),
    Relative(f32),
}

impl SkinWidth {
    fn scaled(&self, shape: &Collider) -> f32 {
        match self {
            SkinWidth::Absolute(value) => *value,
            SkinWidth::Relative(value) => {
                let extents: Vec3 = shape.shape().compute_local_aabb().extents().into();
                value * extents.max_element()
            }
        }
    }
}

struct CollideAndSlideConfig {
    up: Dir3,
    max_slope_angle: f32,
    skin_width: SkinWidth,
    normal_nudge: f32,
    max_slide_count: u32,
}

struct CollideAndSlideHit {
    entity: Entity,
    incoming: Vec3,
    remaining: Vec3,
    slope: Slope,
}

struct CollideAndSlideOutput {
    motion: Vec3,
    remaining: Vec3,
    slope: Option<Slope>,
}

fn collide_and_slide(
    origin: Vec3,
    rotation: Quat,
    target_motion: Vec3,
    config: &CollideAndSlideConfig,
    shape: &Collider,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    mut on_hit: impl FnMut(CollideAndSlideHit),
) -> CollideAndSlideOutput {
    let skin_width = config.skin_width.scaled(shape);

    let mut total_motion = Vec3::ZERO;
    let mut remaining_motion = target_motion;
    let mut last_slope = None;

    for _ in 0..config.max_slide_count {
        if let Ok((direction, length)) = Dir3::new_and_length(remaining_motion) {
            if let Some(hit) = spatial.cast_shape(
                shape,
                origin,
                rotation,
                direction,
                &ShapeCastConfig {
                    max_distance: length + skin_width,
                    target_distance: skin_width, // I don't know what this does, I copied it from rapier.
                    compute_contact_on_penetration: true,
                    ignore_origin_penetration: true,
                },
                filter,
            ) {
                let incoming_motion = direction * (hit.distance - skin_width);

                total_motion += incoming_motion;
                remaining_motion -= incoming_motion;

                let slope = Slope::new(
                    config.up,
                    Dir3::new_unchecked(hit.normal1),
                    config.max_slope_angle,
                );
                last_slope = Some(slope);

                on_hit(CollideAndSlideHit {
                    entity: hit.entity,
                    incoming: incoming_motion,
                    remaining: remaining_motion,
                    slope,
                });
            }
        }

        total_motion += mem::take(&mut remaining_motion);
        break;
    }

    CollideAndSlideOutput {
        motion: total_motion,
        remaining: remaining_motion,
        slope: last_slope,
    }
}

fn depenetrate() {}
