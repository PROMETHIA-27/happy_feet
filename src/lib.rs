use avian3d::{math::PI, parry::shape::TypedShape, prelude::*};
use bevy::{
    color::palettes::css::*,
    ecs::{component::ComponentId, world::DeferredWorld},
    prelude::*,
};
use smallvec::SmallVec;
use std::mem;

// TODO: components and systems for character.

#[derive(Component)]
#[require(PreviousTransform, CalculatedVelocity)]
pub struct MovingPlatform {
    pub start: Vec3,
    pub end: Vec3,
    pub speed: f32,
}

#[derive(Component)]
#[require(PreviousTransform, CalculatedVelocity)]
pub struct RotatingPlatform {
    pub axis: Dir3,
    pub speed: f32,
}

#[derive(Component, Default, Deref, DerefMut)]
#[require(Transform)]
#[component(on_insert = on_insert_prev_trans)]
pub struct PreviousTransform(Transform);

fn on_insert_prev_trans(mut world: DeferredWorld, entity: Entity, _: ComponentId) {
    let transform = *world.get::<Transform>(entity).unwrap();
    let mut prev = world.get_mut::<PreviousTransform>(entity).unwrap();
    prev.0 = transform;
}

#[derive(Component, Default, Clone, Copy)]
pub struct CalculatedVelocity {
    pub linear: Vec3,
    pub angular: Vec3,
}

pub fn update_platform_velocity(
    mut query: Query<(&mut PreviousTransform, &Transform, &mut CalculatedVelocity)>,
    time: Res<Time>,
) {
    for (mut previous_trans, current_trans, mut calculated_velocity) in &mut query {
        calculated_velocity.linear =
            (current_trans.translation - previous_trans.translation) / time.delta_secs();
        previous_trans.translation = current_trans.translation;

        let mut rel_rot = current_trans.rotation * previous_trans.rotation.inverse();
        if rel_rot.w < 0.0 {
            rel_rot = -rel_rot;
        }
        rel_rot = rel_rot.normalize();

        let (mut axis, mut angle) = rel_rot.to_axis_angle();
        if angle > PI {
            angle -= 2.0 * PI;
            axis = -axis;
        }

        calculated_velocity.angular = angle / time.delta_secs() * axis;
        previous_trans.rotation = current_trans.rotation;
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SlopePlane {
    Floor,
    Wall,
    Roof,
}

#[derive(Debug, Clone, Copy)]
pub struct Slope {
    pub up: Dir3,
    pub normal: Dir3,
    pub angle: f32,
    pub plane: SlopePlane,
}

impl Slope {
    pub fn new(up: Dir3, normal: Dir3, max_floor_angle: f32) -> Self {
        let angle = f32::acos(normal.dot(*up));

        if angle > PI / 2.0 {
            Self {
                angle,
                plane: SlopePlane::Roof,
                normal,
                up,
            }
        } else if angle >= max_floor_angle {
            Self {
                angle,
                plane: SlopePlane::Wall,
                normal,
                up,
            }
        } else {
            Self {
                angle,
                plane: SlopePlane::Floor,
                normal,
                up,
            }
        }
    }

    pub fn is_floor(&self) -> bool {
        matches!(self.plane, SlopePlane::Floor)
    }
}

#[derive(Component, Clone, Copy)]
pub struct Floor {
    pub entity: Entity,
    pub normal: Dir3,
}

pub struct MovementOutput {
    pub velocity: Vec3,
    pub motion: Vec3,
    pub remaining_motion: Vec3,
    pub floor: Option<Floor>,
    pub overlap_amount: f32,
}

pub struct MovementConfig {
    pub up_direction: Dir3,
    pub skin_width: f32,
    pub floor_snap_distance: f32,
    pub max_floor_angle: f32,
}

// FIXME:
// - Walking along walls made up of multiple colliders results in sudden changes in speed
// and generally feels "off". I don't know how other engines solves this (if they do).
// - Jumping doesn't work when walking into wall.
pub fn move_and_slide(
    was_on_floor: bool,
    origin: Vec3,
    mut velocity: Vec3,
    rotation: Quat,
    config: MovementConfig,
    shape: &Collider,
    spatial: &SpatialQuery,
    filter: &SpatialQueryFilter,
    delta_secs: f32,
    mut on_collide: impl FnMut(Slope),
    gizmos: &mut Gizmos,
) -> MovementOutput {
    let mut motion = Vec3::ZERO;
    let mut remaining_motion = velocity * delta_secs;

    let inflated_shape = inflate_shape(shape, config.skin_width);

    let mut index = SlopeIndex::First;

    let mut floor = None;

    for _ in 0..12 {
        // 1. Sweep in the movement direction.
        if let Ok((direction, length)) = Dir3::new_and_length(remaining_motion) {
            if let Some(hit) = spatial.cast_shape(
                shape,
                origin + motion,
                rotation,
                direction,
                &ShapeCastConfig {
                    max_distance: length + config.skin_width,
                    target_distance: config.skin_width,
                    compute_contact_on_penetration: true,
                    ignore_origin_penetration: true,
                },
                filter,
            ) {
                let incoming = direction * (hit.distance - config.skin_width);

                // Move as far as possible.
                motion += incoming;
                remaining_motion -= incoming;

                let slope = Slope::new(
                    config.up_direction,
                    Dir3::new_unchecked(hit.normal1),
                    config.max_floor_angle,
                );

                if slope.is_floor() {
                    floor = Some(Floor {
                        entity: hit.entity,
                        normal: slope.normal,
                    })
                } else {
                    project_motion(was_on_floor, &mut index, slope, [
                        &mut remaining_motion,
                        &mut velocity,
                    ]);
                }

                on_collide(slope);
            } else {
                motion += mem::take(&mut remaining_motion);
            }
        }

        // 3. Solve overlaps.
        //
        // We want to be able to climb walkable slopes without slowing down,
        // so we solve those overlaps vertically only.
        if let Some((_, overlaps)) = get_penetration::<8>(
            &inflated_shape,
            origin + motion,
            rotation,
            Dir3::new(motion).unwrap_or(-config.up_direction),
            filter,
            spatial,
        ) {
            let mut vertical = Vec3::ZERO;
            let mut horizontal = Vec3::ZERO;
            let mut rest = Vec3::ZERO;

            for overlap in &overlaps {
                let slope = Slope::new(config.up_direction, overlap.normal, config.max_floor_angle);
                on_collide(slope); // FIXME: this can be called many times on the same wall. 

                if slope.is_floor() {
                    vertical += overlap.normal * overlap.depth;
                    floor = Some(Floor {
                        entity: overlap.entity,
                        normal: overlap.normal,
                    });
                } else if was_on_floor {
                    horizontal += overlap.normal * overlap.depth;
                } else {
                    rest += overlap.normal * overlap.depth;
                }
            }

            motion += vertical.project_onto_normalized(*config.up_direction);
            motion +=
                horizontal - config.up_direction.dot(horizontal).max(0.0) * config.up_direction;
            motion += rest;
        }

        if remaining_motion.length_squared() < 1e-4 {
            break;
        }
    }

    // 4. Solve remaining overlaps.
    //
    // This reduces jitter when walking into walls.
    let mut overlap_amount = 0.0;

    for _ in 0..4 {
        overlap_amount = 0.0;
        if let Some((_fixup, overlaps)) = get_penetration::<8>(
            &inflated_shape,
            origin + motion,
            rotation,
            -config.up_direction,
            filter,
            spatial,
        ) {
            let mut vertical = Vec3::ZERO;
            let mut horizontal = Vec3::ZERO;
            let mut rest = Vec3::ZERO;

            for overlap in overlaps {
                overlap_amount += overlap.depth;

                let slope = Slope::new(config.up_direction, overlap.normal, config.max_floor_angle);
                on_collide(slope);

                if slope.is_floor() {
                    vertical += overlap.normal * overlap.depth;
                } else if was_on_floor {
                    horizontal += overlap.normal * overlap.depth;
                } else {
                    rest += overlap.normal * overlap.depth;
                }
            }

            motion += vertical.project_onto_normalized(*config.up_direction);
            motion +=
                horizontal - config.up_direction.dot(horizontal).max(0.0) * config.up_direction;
            motion += rest;
        } else {
            overlap_amount = 0.0;

            break;
        }
    }

    // 5. Detect and snap to floor.
    if was_on_floor && floor.is_none() {
        spatial.shape_hits_callback(
            &inflated_shape,
            origin + motion,
            rotation,
            -config.up_direction,
            &ShapeCastConfig {
                max_distance: config.floor_snap_distance + 0.01,
                target_distance: 0.0,
                compute_contact_on_penetration: true,
                ignore_origin_penetration: true,
            },
            filter,
            |hit| {
                if hit.distance <= config.floor_snap_distance {
                    motion -= config.up_direction * hit.distance;
                }
                let slope = Slope::new(
                    config.up_direction,
                    Dir3::new_unchecked(hit.normal1),
                    config.max_floor_angle,
                );
                if slope.is_floor() {
                    floor = Some(Floor {
                        entity: hit.entity,
                        normal: slope.normal,
                    })
                }
                floor.is_none()
            },
        );
    }

    MovementOutput {
        velocity,
        motion,
        remaining_motion,
        floor,
        overlap_amount,
    }
}

#[derive(Debug, Clone, Copy)]
enum SlopeIndex {
    First,
    Second { first_normal: Dir3 },
    Third,
}

fn project_motion<'a>(
    was_on_floor: bool,
    index: &mut SlopeIndex,
    slope: Slope,
    motion_vectors: impl IntoIterator<Item = &'a mut Vec3>,
) {
    let new_index = match index {
        SlopeIndex::First => SlopeIndex::Second {
            first_normal: slope.normal,
        },
        SlopeIndex::Second { first_normal } => {
            if first_normal.dot(*slope.normal) < 1.0 - 1e-4 {
                SlopeIndex::Third
            } else {
                // If the new normal is identical to the previous one, then we can assume
                // the motion has already been handled in the previous iteration.
                return;
            }
        }
        SlopeIndex::Third => SlopeIndex::Third,
    };

    for motion in motion_vectors.into_iter() {
        match index {
            // We want to remove all motion in the direction of the hit normal,
            // i.e., project the motion on the normal plane.
            SlopeIndex::First => {
                *motion = motion.reject_from_normalized(*slope.normal);
            }
            // Since the previous iteration changed the motion direction, doing so again might
            // result in a new motion where no parts of it existed in the original velocity.
            // This may result in the character "shaking" when walking into a corner.
            //
            // We solve this by projecting onto a "crease" normal instead.
            SlopeIndex::Second { first_normal } => {
                let crease = first_normal.cross(*slope.normal).normalize();
                *motion = motion.project_onto_normalized(crease);
            }
            // When the body is sliding on three unique slopes, no more motion is possible.
            SlopeIndex::Third => *motion = Vec3::ZERO,
        }

        // Remove vertical motion when the slope is not walkable.
        if was_on_floor && !slope.is_floor() {
            *motion -= *slope.up * motion.dot(*slope.up).max(0.0);
        }
    }

    *index = new_index;
}

#[derive(Debug, Clone, Copy)]
struct OverlapInfo {
    entity: Entity,
    normal: Dir3,
    depth: f32,
    start: Vec3,
    point: Vec3,
    hit: ShapeHitData,
}

#[must_use]
fn get_penetration<const MAX_OVERLAPS: usize>(
    shape: &Collider,
    position: Vec3,
    rotation: Quat,
    direction: Dir3,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
) -> Option<(Vec3, SmallVec<[OverlapInfo; MAX_OVERLAPS]>)> {
    let mut overlaps = SmallVec::<[OverlapInfo; MAX_OVERLAPS]>::new_const();

    // Find all overlapping bodies.
    spatial.shape_hits_callback(
        &shape,
        position,
        rotation,
        direction,
        &ShapeCastConfig {
            max_distance: 0.0,
            target_distance: 0.0,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: false,
        },
        filter,
        |hit| {
            let start = rotation * hit.point2 + position;
            let depth_sq = start.distance_squared(hit.point1);
            let overlap = OverlapInfo {
                entity: hit.entity,
                normal: Dir3::new_unchecked(hit.normal1),
                depth: depth_sq.sqrt(),
                start,
                point: hit.point1,
                hit: hit,
            };
            if depth_sq > 1e-8 {
                overlaps.push(overlap);
            }
            overlaps.capacity() > overlaps.len()
        },
    );

    if overlaps.is_empty() {
        return None;
    }

    // Larger overlaps are more important, so they go first.
    overlaps.sort_by(|a, b| b.depth.total_cmp(&a.depth));

    let mut fixup = Vec3::ZERO;
    let mut total_depth = 0.0;

    // Solve each overlap.
    for i in 0..overlaps.len() {
        let overlap = overlaps[i];

        fixup += overlap.depth * overlap.normal; // Accumulate fixup.
        total_depth += overlap.depth;

        // Make sure the next overlaps doesn't move the character further than neccessary.
        for next_overlap in &mut overlaps[i + 1..] {
            if next_overlap.depth < 1e-4 {
                continue;
            }
            let accounted_for = f32::max(
                0.0,
                overlap.normal.dot(*next_overlap.normal) * overlap.depth,
            );
            next_overlap.depth = f32::max(0.0, next_overlap.depth - accounted_for);
        }
    }

    if total_depth < 1e-4 {
        return None;
    }

    Some((fixup, overlaps))
}

/// Grow or shrink a [`Collider`].
pub fn inflate_shape(shape: &Collider, value: f32) -> Collider {
    match shape.shape().as_typed_shape() {
        TypedShape::Capsule(capsule) => Collider::capsule(capsule.radius + value, capsule.height()),
        TypedShape::Ball(ball) => Collider::sphere(ball.radius + value),
        _ => todo!("implement the remaining shapes that make sense"),
    }
}
