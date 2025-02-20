use std::mem;

use avian3d::{math::PI, parry::shape::TypedShape, prelude::*};
use bevy::{
    color::palettes::css::{BLACK, BLUE, GREEN, LIGHT_BLUE, RED, WHITE},
    math::VectorSpace,
    prelude::*,
};

pub const HALF_PI: f32 = 1.618033988749894848204586834365638118_f32;

#[derive(Clone, Copy)]
enum SlopePlane {
    Floor,
    Wall,
    Roof,
}

#[derive(Clone, Copy)]
pub struct Slope {
    pub normal: Dir3,
    pub angle: f32,
}

impl Slope {
    fn new(up: Dir3, normal: Dir3) -> Self {
        let angle = f32::acos(normal.dot(*up));
        Self { normal, angle }
    }

    pub fn is_roof(&self) -> bool {
        self.angle >= HALF_PI
    }

    pub fn is_wall(&self, max_angle: f32) -> bool {
        self.angle > max_angle && self.angle < HALF_PI
    }

    pub fn is_floor(&self, max_angle: f32) -> bool {
        self.angle <= max_angle
    }

    fn plane(&self, max_angle: f32) -> SlopePlane {
        match self.angle {
            a if a > HALF_PI => SlopePlane::Roof,
            a if a > max_angle => SlopePlane::Wall,
            _ => SlopePlane::Floor,
        }
    }
}

#[derive(Clone, Copy)]
pub struct CollideAndSlideCollision {
    pub entity: Entity,
    /// The motion of the character in this slide step.
    pub incoming: Vec3,
    /// The remaining motion of the character.
    pub remaining: Vec3,
    pub slope: Slope,
}

impl CollideAndSlideCollision {
    pub fn normal(&self) -> Dir3 {
        self.slope.normal
    }
}

pub enum SkinWidth {
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

pub struct CollideAndSlideConfig {
    pub up: Dir3,
    pub max_slope_angle: f32,
    pub skin_width: SkinWidth,
    pub normal_nudge: f32, // TODO: implement
    pub max_slide_count: u32,
    /// Applies the remaining motion to the motion vector when no collision was made.
    /// This should be `false` when using [`LinearVelocity`] to avoid moving at double speed.
    pub apply_remaining_motion: bool,
}

impl Default for CollideAndSlideConfig {
    fn default() -> Self {
        Self {
            up: Dir3::Y,
            max_slope_angle: PI / 2.0,
            skin_width: SkinWidth::Relative(0.01),
            normal_nudge: 1e-4,
            max_slide_count: 5,
            apply_remaining_motion: false,
        }
    }
}

pub struct CollideAndSlideOutput {
    pub motion: Vec3,
    pub remaining: Vec3,
}

#[must_use]
pub fn collide_and_slide2(
    origin: Vec3,
    rotation: Quat,
    target_motion: Vec3,
    config: &CollideAndSlideConfig,
    shape: &Collider,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    mut on_hit: impl FnMut(&ShapeHitData),
    gizmos: &mut Gizmos,
) -> Vec3 {
    // TODO:
    // - [ ] solve all overlaps in one go
    // - [ ] Snap to floor
    // - [ ] stair stepping (steal from rapier)
    // - [ ] slope handling (steal from rapier)

    let skin_width = config.skin_width.scaled(shape);

    let mut position = origin;
    let mut remaining_motion = target_motion;

    enum CollisionPlane {
        First,
        Second { first_plane: Vec3 },
        Third,
    }

    let mut collision_plane = CollisionPlane::First;

    for _ in 0..config.max_slide_count {
        let Ok((direction, length)) = Dir3::new_and_length(remaining_motion) else {
            break;
        };

        gizmos.line(position, position + remaining_motion, WHITE);

        let Some(hit) = spatial.cast_shape(
            shape,
            position,
            rotation,
            direction,
            &ShapeCastConfig {
                max_distance: length + skin_width,
                target_distance: skin_width, // I don't know what this does but rapier did the same.
                compute_contact_on_penetration: true,
                ignore_origin_penetration: true,
            },
            filter,
        ) else {
            position += mem::take(&mut remaining_motion);
            break;
        };

        on_hit(&hit);

        gizmos.arrow(hit.point1, hit.point1 + hit.normal1 * 4.0, RED);

        let incoming = direction * (hit.distance - skin_width); // Move away from surface
        // let incoming = direction * f32::max(0.0, hit.distance - skin_width);

        position += incoming;
        remaining_motion -= incoming + hit.normal1 * config.normal_nudge; // Rapier uses normal nudge, idk why

        match collision_plane {
            // One plane.
            CollisionPlane::First => {
                remaining_motion = remaining_motion.reject_from(hit.normal1);

                collision_plane = CollisionPlane::Second {
                    first_plane: hit.normal1,
                };
            }
            // Two planes, project on crease normal.
            CollisionPlane::Second { first_plane } => {
                let crease = Dir3::new(first_plane.cross(hit.normal1)).unwrap();

                remaining_motion = remaining_motion.project_onto(*crease);

                gizmos.arrow(hit.point1, hit.point1 + crease * 4.0, LIGHT_BLUE);

                collision_plane = CollisionPlane::Third;
            }
            // Three planes, no motion is possible.
            CollisionPlane::Third => remaining_motion = Vec3::ZERO,
        }
    }

    let mut hits = Vec::with_capacity(8);

    // Solve overlaps.
    // 16 iterations are worst case scenario, 1 is usually enough.
    for i in 0..16 {
        hits.clear();
        spatial.shape_hits_callback(
            &inflate_shape(shape, skin_width),
            position,
            rotation,
            -config.up,
            &ShapeCastConfig {
                max_distance: 0.0,    // Use this for snaping to floor.
                target_distance: 0.0, // I don't know what this does, I copied it from rapier.
                compute_contact_on_penetration: true,
                ignore_origin_penetration: false,
            },
            &filter,
            |mut hit| {
                let start = rotation * hit.point2 + position;
                let dist_sq = start.distance_squared(hit.point1);
                if dist_sq > 1e-4 {
                    hit.distance = dist_sq.sqrt();
                    hits.push(hit);
                }

                hits.capacity() > hits.len()
            },
        );

        if hits.is_empty() {
            if i > 0 {
                println!("solved collisions in {} iterations", i);
            }
            break;
        }

        // Largest overlap goes first. Is this neccessary?
        hits.sort_by(|a, b| b.distance.total_cmp(&a.distance));

        // let total = hits
        //     .iter()
        //     .fold(Vec3::ZERO, |acc, hit| acc + hit.normal1 * hit.distance);
        // position += total / hits.len() as f32;

        // let hit = &hits[0];
        // position += hit.distance * hit.normal1;

        // Iteratively solve each hit.
        for i in 0..hits.len() {
            let hit = hits[i];

            if hit.distance < 1e-4 {
                continue;
            }

            on_hit(&hit);

            position += hit.distance * hit.normal1;

            // Make sure the next hits doesn't uneccessarily move the character.
            for next_hit in &mut hits[i + 1..] {
                let accounted_for = f32::max(0.0, hit.normal1.dot(next_hit.normal1) * hit.distance);
                next_hit.distance = f32::max(0.0, next_hit.distance - accounted_for);
            }
        }
    }

    position
}

#[must_use]
pub fn collide_and_slide(
    origin: Vec3,
    rotation: Quat,
    target_motion: Vec3,
    config: &CollideAndSlideConfig,
    shape: &Collider,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    mut on_hit: impl FnMut(&CollideAndSlideCollision),
    gizmos: &mut Gizmos, // TODO: remove this
) -> CollideAndSlideOutput {
    let skin_width = config.skin_width.scaled(shape);

    let mut total_motion = Vec3::ZERO;
    let mut remaining_motion = target_motion;

    let mut last_collision: Option<CollideAndSlideCollision> = None;

    // return CollideAndSlideOutput {
    //     motion: target_motion
    //         + depenetrate(
    //             origin,
    //             rotation,
    //             target_motion,
    //             skin_width,
    //             shape,
    //             filter,
    //             spatial,
    //             gizmos,
    //         ),
    //     remaining: Vec3::ZERO,
    // };

    // let fixup = depenetrate(
    //     origin, rotation, skin_width, 8, shape, filter, spatial, gizmos,
    // );

    // remaining_motion += fixup;

    // if penetration.length_squared() > 1e-4 {
    //     remaining_motion += dbg!(penetration);
    // }

    // return CollideAndSlideOutput {
    //     motion: target_motion + fixup,
    //     remaining: Vec3::ZERO,
    // };

    // total_motion += depenetrate(
    //     origin, rotation, skin_width, 16, shape, filter, spatial, gizmos,
    // );

    for _ in 0..config.max_slide_count {
        let Some(collision) = collide(
            origin + total_motion,
            rotation,
            remaining_motion,
            config.up,
            skin_width,
            shape,
            filter,
            spatial,
            gizmos,
        ) else {
            if config.apply_remaining_motion {
                total_motion += mem::take(&mut remaining_motion);
            }
            break;
        };

        total_motion += collision.incoming;
        // remaining_motion = collision.remaining;

        on_hit(&collision);

        // remaining_motion = remaining_motion.reject_from(*collision.normal())
        //     - collision.normal() * config.normal_nudge;

        match last_collision {
            Some(last_collision) => {
                let crease = last_collision
                    .normal()
                    .cross(*collision.normal())
                    .normalize();
                // gizmos.arrow(, end, color)
                remaining_motion = collision.remaining.project_onto(crease);
            }
            _ => {
                remaining_motion = collision.remaining.reject_from(*collision.normal());
            }
        }

        last_collision = Some(collision);

        // total_motion += depenetrate(
        //     origin, rotation, skin_width, 16, shape, filter, spatial, gizmos,
        // );

        // match depen_and_collide(
        //     origin,
        //     rotation,
        //     target_motion,
        //     config.up,
        //     skin_width,
        //     16,
        //     shape,
        //     filter,
        //     spatial,
        //     gizmos,
        // ) {
        //     Ok(collision) => {
        //         on_hit(&collision);

        //         total_motion += collision.incoming;
        //         remaining_motion = collision.remaining.reject_from(*collision.normal())
        //         // - collision.normal() * config.normal_nudge;
        //     }
        //     Err(fixup) => {
        //         total_motion += fixup;

        //         if config.apply_remaining_motion {
        //             total_motion += mem::take(&mut remaining_motion);
        //         }

        //         break;
        //     }
        // }
    }

    CollideAndSlideOutput {
        motion: total_motion,
        remaining: remaining_motion,
    }
}

fn collide(
    origin: Vec3,
    rotation: Quat,
    target_motion: Vec3,
    up: Dir3,
    skin_width: f32,
    shape: &Collider,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    gizmos: &mut Gizmos, // TODO: remove this
) -> Option<CollideAndSlideCollision> {
    // Don't even bother.
    if target_motion.length_squared() < 1e-6 {
        return None;
    }

    let (direction, length) = Dir3::new_and_length(target_motion).ok()?;

    gizmos.line(origin, origin + target_motion, WHITE);
    let hit = spatial.cast_shape(
        shape,
        origin,
        rotation,
        direction,
        &ShapeCastConfig {
            // max_distance: length,
            // target_distance: 0.0,
            max_distance: length + skin_width,
            target_distance: skin_width, // I don't know what this does, I copied it from rapier.
            compute_contact_on_penetration: true,
            ignore_origin_penetration: true,
        },
        filter,
    )?;
    gizmos.arrow(hit.point1, hit.point1 + hit.normal1 * 4.0, RED);

    let incoming = direction * (hit.distance - skin_width);

    let slope = Slope::new(up, Dir3::new_unchecked(hit.normal1));

    Some(CollideAndSlideCollision {
        entity: hit.entity,
        incoming,
        remaining: target_motion - incoming,
        slope,
    })
}

#[must_use]
fn depenetrate(
    origin: Vec3,
    rotation: Quat,
    skin_width: f32,
    max_iterations: usize,
    shape: &Collider,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    gizmos: &mut Gizmos, // TODO: remove this
) -> Vec3 {
    let inflated_shape = inflate_shape(shape, skin_width);

    let hits = spatial.shape_hits(
        &inflated_shape,
        origin,
        rotation,
        Dir3::NEG_Y,
        16,
        &ShapeCastConfig {
            max_distance: 0.0,
            target_distance: 0.0,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: false,
        },
        filter,
    );

    let mut fixup = Vec3::ZERO;

    for _ in 0..max_iterations {
        let mut accum_error = 0.0;
        for hit in &hits {
            if hit.distance != 0.0 {
                continue;
            }
            let start = rotation * hit.point2 + origin;
            let end = hit.point1;

            let penetration = end - start;
            let depth = penetration.length();

            gizmos.cross(Isometry3d::from_translation(start), 2.0, RED);
            gizmos.cross(Isometry3d::from_translation(end), 2.0, BLUE);

            // let error = f32::max(0.0, (depth + skin_width) - fixup.dot(hit.normal1));
            let error = f32::max(0.0, depth - fixup.dot(hit.normal1));
            accum_error += error;
            fixup += error * hit.normal1;
        }

        if accum_error < 1e-4 {
            break;
        }
    }

    gizmos.line(origin, origin + fixup, RED);

    let extents: Vec3 = shape.shape().compute_local_aabb().extents().into();
    fixup = fixup.clamp_length_max(extents.max_element());

    gizmos.sphere(Isometry3d::from_translation(origin + fixup), 1.0, GREEN);

    fixup
}

fn depen_and_collide(
    origin: Vec3,
    rotation: Quat,
    target_motion: Vec3,
    up: Dir3,
    skin_width: f32,
    max_iterations: usize,
    shape: &Collider,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    gizmos: &mut Gizmos, // TODO: remove this
) -> Result<CollideAndSlideCollision, Vec3> {
    let inflated_shape = inflate_shape(shape, skin_width);

    let dir_and_length = Dir3::new_and_length(target_motion).ok();

    let hits = {
        let (direction, length) = dir_and_length.unwrap_or((Dir3::NEG_Y, 0.0));
        spatial.shape_hits(
            &inflated_shape,
            origin,
            rotation,
            direction,
            16,
            &ShapeCastConfig {
                max_distance: length,
                target_distance: skin_width, // Again, not sure what this does.
                compute_contact_on_penetration: true,
                ignore_origin_penetration: false,
            },
            filter,
        )
    };

    // If no hits are penetrating at origin.
    if hits.iter().all(|hit| hit.distance > 0.0) {
        let Some(hit) = hits.last() else {
            return Err(Vec3::ZERO);
        };

        let (direction, _) = dir_and_length.unwrap();
        let incoming = direction * hit.distance;

        let slope = Slope::new(up, Dir3::new_unchecked(hit.normal1));

        return Ok(CollideAndSlideCollision {
            entity: hit.entity,
            incoming,
            remaining: target_motion - incoming,
            slope,
        });
    }

    let mut fixup = Vec3::ZERO;

    for _ in 0..max_iterations {
        let mut accum_error = 0.0;
        for hit in &hits {
            if hit.distance != 0.0 {
                continue;
            }
            let start = rotation * hit.point2 + origin;
            let end = hit.point1;

            let penetration = end - start;
            let depth = penetration.length();

            gizmos.cross(Isometry3d::from_translation(start), 2.0, RED);
            gizmos.cross(Isometry3d::from_translation(end), 2.0, BLUE);

            // let error = f32::max(0.0, (depth + skin_width) - fixup.dot(hit.normal1));
            let error = f32::max(0.0, depth - fixup.dot(hit.normal1));
            accum_error += error;
            fixup += error * hit.normal1;
        }

        if accum_error < 1e-4 {
            break;
        }
    }

    gizmos.line(origin, origin + fixup, RED);

    let extents: Vec3 = shape.shape().compute_local_aabb().extents().into();
    fixup = fixup.clamp_length_max(extents.max_element());

    gizmos.sphere(Isometry3d::from_translation(origin + fixup), 1.0, GREEN);

    // let Some((direction, length)) = dir_and_length else {
    //     return Err(fixup);
    // };

    // let origin = origin + fixup;
    // let target_motion = target_motion + fixup;

    // let Ok((direction, length)) = Dir3::new_and_length(target_motion) else {
    //     return Err(fixup);
    // };

    // if let Some(hit) = spatial.cast_shape(
    //     shape,
    //     origin + fixup,
    //     rotation,
    //     direction,
    //     &ShapeCastConfig {
    //         max_distance: length + skin_width,
    //         target_distance: skin_width, // Again, not sure what this does.
    //         compute_contact_on_penetration: true,
    //         ignore_origin_penetration: false,
    //     },
    //     filter,
    // ) {
    //     let incoming = direction * (hit.distance - skin_width);

    //     let slope = Slope::new(up, Dir3::new_unchecked(hit.normal1));

    //     return Ok(CollideAndSlideCollision {
    //         entity: hit.entity,
    //         incoming: incoming,
    //         remaining: target_motion - incoming,
    //         slope,
    //     });
    // }

    // return Err(fixup);

    if let Some(collision) = collide(
        origin + fixup,
        rotation,
        target_motion + fixup,
        up,
        skin_width,
        shape,
        filter,
        spatial,
        gizmos,
    ) {
        Ok(collision)
    } else {
        Err(fixup)
    }
}

fn inflate_shape(shape: &Collider, amount: f32) -> Collider {
    match shape.shape().as_typed_shape() {
        TypedShape::Capsule(capsule) => {
            Collider::capsule(capsule.radius + amount, capsule.height())
        }
        _ => unimplemented!(),
    }
}
