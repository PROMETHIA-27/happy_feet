use std::{mem, ops::DerefMut};

use avian3d::{math::PI, parry::shape::TypedShape, prelude::*};
use bevy::{
    color::palettes::css::*,
    ecs::{component::ComponentId, world::DeferredWorld},
    prelude::*,
};
use smallvec::{Array, SmallVec};

struct Character {
    floor_entity: Option<Entity>,
}

struct OnStep;

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
enum SlopePlane {
    Floor,
    Wall,
    Roof,
}

#[derive(Debug, Clone, Copy)]
struct Slope {
    normal: Dir3,
    up: Dir3,
    angle: f32,
    plane: SlopePlane,
}

impl Slope {
    fn new(up: Dir3, normal: Dir3, max_floor_angle: f32) -> Self {
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

    fn is_floor(&self) -> bool {
        matches!(self.plane, SlopePlane::Floor)
    }
}

#[derive(Component, Clone, Copy)]
pub struct Floor {
    pub entity: Entity,
    pub normal: Dir3,
}

pub struct MovementOutput {
    pub position: Vec3,
    pub velocity: Vec3,
    pub remaining_motion: Vec3,
    pub floor: Option<Floor>,
    pub is_on_wall: bool,
    pub is_on_roof: bool,
}

pub struct MovementConfig {
    pub up_direction: Dir3,
    pub skin_width: f32,
    pub floor_snap_distance: f32,
    pub max_floor_angle: f32,
}

pub fn move_character(
    was_on_floor: bool,
    origin: Vec3,
    mut velocity: Vec3,
    rotation: Quat,
    config: MovementConfig,
    shape: &Collider,
    spatial: &SpatialQuery,
    filter: &SpatialQueryFilter,
    delta_secs: f32,
) -> (Vec3, Vec3, Option<Floor>) {
    let mut motion = Vec3::ZERO;
    let mut remaining_motion = velocity * delta_secs;

    let inflated_shape = inflate_shape(shape, config.skin_width);

    let mut slope_index = SlopeIndex::First;

    let mut floor = None;

    for _ in 0..16 {
        let mut initial_slope = None;

        // Sweep in the movement direction.
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
                let incoming = direction * (hit.distance - config.skin_width); // Tiny offset away from the surface.

                // Move as far as possible.
                motion += incoming;
                remaining_motion -= incoming;

                let slope = Slope::new(
                    config.up_direction,
                    Dir3::new_unchecked(hit.normal1),
                    config.max_floor_angle,
                );

                initial_slope = Some(slope);

                if slope.is_floor() {
                    floor = Some(Floor {
                        entity: hit.entity,
                        normal: slope.normal,
                    });
                }
            } else {
                motion += mem::take(&mut remaining_motion);
            }
        }

        // Solve overlaps vertically.
        if let Some((fixup, _overlaps)) = solve_overlaps::<4>(
            &inflated_shape,
            origin + motion,
            rotation,
            -config.up_direction,
            filter,
            spatial,
            |overlap| {
                let slope = Slope::new(config.up_direction, overlap.normal, config.max_floor_angle);
                if slope.is_floor() {
                    floor = Some(Floor {
                        entity: overlap.entity,
                        normal: overlap.normal,
                    });
                    return true;
                }
                false
            },
        ) {
            motion += fixup.project_onto_normalized(*config.up_direction);
        } else if let Some(slope) = initial_slope {
            handle_slope(was_on_floor, &mut slope_index, slope, [
                &mut remaining_motion,
                &mut velocity,
            ]);
        }
    }

    // Solve remaining overlaps.
    for _ in 0..8 {
        if let Some((fixup, overlaps)) = solve_overlaps::<8>(
            &inflated_shape,
            origin + motion,
            rotation,
            -config.up_direction,
            filter,
            spatial,
            |_| true,
        ) {
            // Don't push character up when grounded.
            if was_on_floor {
                motion += fixup - config.up_direction.dot(fixup).max(0.0) * config.up_direction;
            } else {
                motion += fixup;
            }

            // Handle slopes.
            for overlap in overlaps {
                let slope = Slope::new(config.up_direction, overlap.normal, config.max_floor_angle);
                if slope.is_floor() {
                    floor = Some(Floor {
                        entity: overlap.entity,
                        normal: overlap.normal,
                    });
                } else {
                    handle_slope(was_on_floor, &mut slope_index, slope, [&mut velocity]);
                }
            }
        } else {
            break;
        }
    }

    // Detect floor state.
    //
    // FIXME: snaps to floor after jumping
    if was_on_floor && floor.is_none() {
        spatial.shape_hits_callback(
            &inflated_shape,
            origin + motion,
            rotation,
            -config.up_direction,
            &ShapeCastConfig {
                max_distance: config.floor_snap_distance,
                target_distance: 0.0,
                compute_contact_on_penetration: true,
                ignore_origin_penetration: true,
            },
            filter,
            |hit| {
                motion -= config.up_direction * hit.distance;
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

    (motion, velocity, floor)
}

// FIXME:
// - character can get stuck on sharp edges while moving in the air.
// - cannot jump while floor_snap_distance > 0.0,
// event when was_on_floor is set to false.
pub fn collide_and_slide(
    was_on_floor: bool,
    mut position: Vec3,
    rotation: Quat,
    mut velocity: Vec3,
    up_direction: Dir3,
    skin_width: f32,
    max_floor_angle: f32,
    max_step_height: f32,
    min_step_depth: f32,
    floor_snap_distance: f32,
    climb_walkable_slopes: bool,
    should_solve_overlaps: bool,
    apply_remaining_motion: bool,
    shape: &Collider,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    delta_time: f32,
    gizmos: &mut Gizmos,
) -> MovementOutput {
    let inflated_shape = inflate_shape(shape, skin_width);

    let mut remaining_motion = velocity * delta_time;

    let mut floor: Option<Floor> = None;
    let mut is_on_wall = false;
    let mut is_on_roof = false;

    let mut slope_index = SlopeIndex::First;

    for _ in 0..16 {
        let Ok((direction, length)) = Dir3::new_and_length(remaining_motion) else {
            break;
        };

        // 1. Climb walkable slopes, also snap to floor.
        // This is done to avoid losing speed when walking up slopes and ledges.
        if was_on_floor && climb_walkable_slopes {
            let offset = skin_width + 1e-4;
            let step_up = up_direction * offset;
            let step_forward = direction * length;

            // Sweep down from the target step location.
            let max_hits = 8;
            let mut i = 0;
            spatial.shape_hits_callback(
                &inflated_shape,
                position + step_up + step_forward,
                rotation,
                -up_direction,
                &ShapeCastConfig {
                    max_distance: floor_snap_distance + offset,
                    target_distance: skin_width,
                    compute_contact_on_penetration: true,
                    ignore_origin_penetration: true,
                },
                filter,
                |hit| {
                    i += 1;

                    let slope = Slope::new(
                        up_direction,
                        Dir3::new_unchecked(hit.normal1),
                        max_floor_angle,
                    );

                    // TODO: this may need some additional checks to make sure the character
                    // can actually stand here.
                    if slope.is_floor() {
                        let incoming = -up_direction * hit.distance; // FIXME: this may make it possible to step through walls?
                        let new_position = position + step_up + step_forward + incoming;

                        let initial_hit = hit.distance <= offset;
                        let below_body =
                            hit.point1.dot(*up_direction) < new_position.dot(*up_direction);

                        if below_body {
                            floor = Some(Floor {
                                entity: hit.entity,
                                normal: Dir3::new_unchecked(hit.normal1),
                            });

                            position = new_position;
                            remaining_motion -= step_forward;

                            gizmos.sphere(Isometry3d::from_translation(hit.point1), 1., WHITE);
                        } else {
                            gizmos.sphere(Isometry3d::from_translation(hit.point1), 1., RED);
                        }

                        return false;
                    }

                    gizmos.sphere(Isometry3d::from_translation(hit.point1), 1., RED);

                    i < max_hits
                },
            );
        }

        // Sweep for collisions in movement direction.
        let Some(hit) = spatial.cast_shape(
            shape,
            position,
            rotation,
            direction,
            &ShapeCastConfig {
                max_distance: length + skin_width,
                target_distance: skin_width,
                compute_contact_on_penetration: true,
                ignore_origin_penetration: true,
            },
            filter,
        ) else {
            // If nothing was hit, it's safe to apply remaining motion.
            if apply_remaining_motion {
                position += mem::take(&mut remaining_motion);
            }
            break;
        };

        let incoming = direction * (hit.distance - skin_width); // Move a little bit away from the surface.

        // Move to the hit point.
        position += incoming;
        remaining_motion -= incoming;

        let slope = Slope::new(
            up_direction,
            Dir3::new_unchecked(hit.normal1),
            max_floor_angle,
        );

        match slope.plane {
            SlopePlane::Floor => {
                floor = Some(Floor {
                    entity: hit.entity,
                    normal: Dir3::new_unchecked(hit.normal1),
                })
            }
            SlopePlane::Wall => is_on_wall = true,
            SlopePlane::Roof => is_on_roof = true,
        }

        // 2. Attempt to step over walls.
        //
        // FIXME: This does not work for stepping over walls at extreme angles.
        let mut did_step = false;
        if max_step_height > 0.0 && !slope.is_floor() {
            if let Ok((horizontal_dir, horizontal_len)) =
                Dir3::new_and_length(remaining_motion.reject_from(*up_direction))
            {
                let step_depth = min_step_depth + skin_width;
                {
                    let step_up = up_direction * max_step_height;
                    let step_forward_dist = step_depth;
                    let step_forward = horizontal_dir * step_forward_dist;

                    // Sweep down from the target step location.
                    if let Some(hit) = spatial.cast_shape(
                        shape,
                        position + step_up + step_forward,
                        rotation,
                        -up_direction,
                        &ShapeCastConfig {
                            max_distance: max_step_height + skin_width,
                            target_distance: skin_width,
                            compute_contact_on_penetration: true,
                            ignore_origin_penetration: true,
                        },
                        filter,
                    ) {
                        // Only step if the slope is walkable.
                        let slope = Slope::new(
                            up_direction,
                            Dir3::new_unchecked(hit.normal1),
                            max_floor_angle,
                        );
                        if slope.is_floor() {
                            let incoming = -up_direction * (hit.distance - skin_width);
                            let new_position = position + step_up + step_forward + incoming;

                            // Place body on the ground after stepping.
                            if let Some(hit) = spatial.cast_shape(
                                &inflated_shape,
                                new_position + up_direction * 1e-4,
                                rotation,
                                -up_direction,
                                &ShapeCastConfig {
                                    max_distance: 10.0,
                                    target_distance: skin_width,
                                    compute_contact_on_penetration: true,
                                    ignore_origin_penetration: true,
                                },
                                filter,
                            ) {
                                position = new_position - up_direction * hit.distance;

                                remaining_motion -=
                                    horizontal_dir * f32::min(step_forward_dist, horizontal_len);

                                did_step = true;
                            }
                        }
                    }
                }
            }
        }

        // 3. Slide if no step was possible.
        //
        // FIXME: It's possible to slowly climb non walkable slopes for some reason.
        if !did_step {
            project_motion_on_slope(
                floor.is_some(),
                &mut slope_index,
                Dir3::new_unchecked(hit.normal1),
                up_direction,
                max_floor_angle,
                [&mut remaining_motion, &mut velocity],
            );
        }
    }

    // 4. Snap to floor.
    //
    // FIXME: this can lead to sliding down corners rather than falling when moving off ledges.
    // Unreal engine has an option that fixes this but introduces a new issue if I remember
    // correctly, might be worth looking into.
    // if floor.is_none() && floor_snap_distance > 0.0
    // if was_on_floor && floor.is_none() {
    //     if let Some(hit) = spatial.cast_shape(
    //         &inflated_shape,
    //         position,
    //         rotation,
    //         -up_direction,
    //         &ShapeCastConfig {
    //             max_distance: floor_snap_distance + skin_width,
    //             target_distance: skin_width,
    //             compute_contact_on_penetration: true,
    //             ignore_origin_penetration: true,
    //         },
    //         filter,
    //     ) {
    //         let slope = SlopePlane::new(
    //             up_direction,
    //             Dir3::new_unchecked(hit.normal1),
    //             max_floor_angle,
    //         );
    //         if slope.is_floor() {
    //             if floor_snap_distance > 0.0 && hit.distance <= floor_snap_distance {
    //                 let incoming = -up_direction * hit.distance;
    //                 position += incoming;
    //             }

    //             floor = Some(Floor {
    //                 entity: hit.entity,
    //                 normal: Dir3::new_unchecked(hit.normal1),
    //             });
    //         }
    //     }
    // }

    // 5. Solve remaining overlaps.
    if should_solve_overlaps {
        let mut overlaps = SmallVec::<[_; 8]>::new_const();
        for _ in 0..16 {
            overlaps.clear();
            if let Some(motion) = solve_overlaps_old(
                &inflated_shape,
                position,
                rotation,
                up_direction,
                max_floor_angle,
                &mut overlaps,
                filter,
                spatial,
                |overlap| {
                    let slope = Slope::new(up_direction, overlap.normal, max_floor_angle);
                    match slope.plane {
                        SlopePlane::Floor => {
                            // FIXME: this is wrong.
                            overlap.normal.reject_from(*up_direction) * overlap.depth
                        }
                        SlopePlane::Wall | SlopePlane::Roof => overlap.normal * overlap.depth,
                    }
                },
            ) {
                position += motion;

                for overlap in &overlaps {
                    if overlap.depth > 1e-4 {
                        project_motion_on_slope(
                            floor.is_some(),
                            &mut slope_index,
                            overlap.normal,
                            up_direction,
                            max_floor_angle,
                            [&mut velocity],
                        );

                        let slope = Slope::new(up_direction, overlap.normal, max_floor_angle);
                        if slope.is_floor() {
                            floor = Some(Floor {
                                entity: overlap.entity,
                                normal: overlap.normal,
                            });
                        }
                    }
                }
            } else {
                break; // No more overlaps.
            }
        }
    }

    MovementOutput {
        position,
        velocity,
        remaining_motion,
        floor,
        is_on_wall,
        is_on_roof,
    }
}

#[derive(Debug, Clone, Copy)]
enum SlopeIndex {
    First,
    Second { first_normal: Dir3 },
    Third,
}

fn handle_slope<'a>(
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

fn project_motion_on_slope<'a>(
    is_on_floor: bool,
    index: &mut SlopeIndex,
    normal: Dir3,
    up_direction: Dir3,
    max_floor_angle: f32,
    motion_vectors: impl IntoIterator<Item = &'a mut Vec3>,
) {
    let new_index = match index {
        SlopeIndex::First => SlopeIndex::Second {
            first_normal: normal,
        },
        SlopeIndex::Second { first_normal } => {
            if first_normal.dot(*normal) < 1.0 - 1e-4 {
                SlopeIndex::Third
            } else {
                // If the new normal is identical to the previous one, then we can assume
                // the motion has already been handled in the previous iteration.
                return;
            }
        }
        SlopeIndex::Third => SlopeIndex::Third,
    };

    let slope = Slope::new(up_direction, normal, max_floor_angle);

    for motion in motion_vectors.into_iter() {
        match index {
            // We want to remove all motion in the direction of the hit normal,
            // i.e., project the motion on the normal plane.
            SlopeIndex::First => {
                *motion = motion.reject_from_normalized(*normal);
            }
            // Since the previous iteration changed the motion direction, doing so again might
            // result in a new motion where no parts of it existed in the original velocity.
            // This may result in the character "shaking" when walking into a corner.
            //
            // We solve this by projecting onto a "crease" normal instead.
            SlopeIndex::Second { first_normal } => {
                let crease = first_normal.cross(*normal).normalize();
                *motion = motion.project_onto_normalized(crease);
            }
            // When the body is sliding on three unique slopes, no more motion is possible.
            SlopeIndex::Third => *motion = Vec3::ZERO,
        }

        // Remove vertical motion when the slope is not walkable.
        if is_on_floor && !slope.is_floor() {
            *motion -= *up_direction * motion.dot(*up_direction).max(0.0);
        }
    }

    *index = new_index;
}

#[derive(Debug, Clone, Copy)]
struct OverlapInfo {
    entity: Entity,
    normal: Dir3,
    depth: f32,
}

#[must_use]
fn solve_overlaps<const MAX_OVERLAPS: usize>(
    inflated_shape: &Collider,
    position: Vec3,
    rotation: Quat,
    direction: Dir3,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    mut f: impl FnMut(&OverlapInfo) -> bool,
) -> Option<(Vec3, SmallVec<[OverlapInfo; MAX_OVERLAPS]>)> {
    let mut overlaps = SmallVec::<[OverlapInfo; MAX_OVERLAPS]>::new_const();

    // Find all overlapping bodies.
    spatial.shape_hits_callback(
        &inflated_shape,
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
            };
            if depth_sq > 1e-8 && f(&overlap) {
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

    // Solve each overlap.
    let mut i = 0;
    while i < overlaps.len() {
        let overlap = overlaps[i];

        if overlap.depth < 1e-4 {
            overlaps.swap_remove(i); // Remove overlap if depth is nearly 0.
            continue;
        }

        fixup += overlap.depth * overlap.normal; // Accumulate fixup.

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

        i += 1;
    }

    if overlaps.is_empty() {
        return None;
    }

    Some((fixup, overlaps))
}

#[must_use]
fn solve_overlaps_old<T: Array<Item = OverlapInfo>>(
    inflated_shape: &Collider,
    position: Vec3,
    rotation: Quat,
    up_direction: Dir3,
    max_floor_angle: f32,
    overlaps: &mut SmallVec<T>,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    mut f: impl FnMut(&OverlapInfo) -> Vec3,
) -> Option<Vec3> {
    assert!(overlaps.is_empty());

    // Find all overlapping bodies.
    spatial.shape_hits_callback(
        &inflated_shape,
        position,
        rotation,
        -up_direction,
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
            if depth_sq > 1e-8 {
                overlaps.push(OverlapInfo {
                    entity: hit.entity,
                    normal: Dir3::new_unchecked(hit.normal1),
                    depth: depth_sq.sqrt(),
                });
            }
            overlaps.capacity() > overlaps.len()
        },
    );

    if overlaps.is_empty() {
        return None;
    }

    // Larger overlaps are more important, so they go first.
    overlaps.sort_by(|a, b| b.depth.total_cmp(&a.depth));

    let mut motion = Vec3::ZERO;

    let mut total_depth = 0.0;

    // Solve each overlap.
    for i in 0..overlaps.len() {
        let overlap = overlaps[i];

        if overlap.depth < 1e-4 {
            continue;
        }

        // Don't move the body up if the slope is not walkable.
        //
        // TODO: remove all of this xd
        // let offset_vector = match Slope::new(up_direction, overlap.normal, max_floor_angle).plane {
        //     SlopePlane::Floor => *overlap.normal,
        //     SlopePlane::Wall | SlopePlane::Roof => {
        //         let up_amount = up_direction.dot(*overlap.normal).max(0.0);
        //         *overlap.normal - up_direction * up_amount
        //     }
        // };

        let Ok((dir, depth)) = Dir3::new_and_length(f(&overlap)) else {
            continue;
        };

        // total_depth += overlap.depth;
        // motion += overlap.depth * offset_vector;

        total_depth += depth;
        motion += dir * depth;

        // Make sure the next overlaps doesn't move the character further than neccessary.

        for next_overlap in &mut overlaps[i + 1..] {
            if next_overlap.depth < 1e-4 {
                continue;
            }
            // let accounted_for =
            //     f32::max(0.0, offset_vector.dot(*next_overlap.normal) * overlap.depth);
            let accounted_for = f32::max(0.0, dir.dot(*next_overlap.normal) * depth);
            next_overlap.depth = f32::max(0.0, next_overlap.depth - accounted_for);
        }
    }

    if total_depth < 1e-4 {
        return None;
    }

    Some(motion)
}

/// Grow or shrink a [`Collider`].
pub fn inflate_shape(shape: &Collider, value: f32) -> Collider {
    match shape.shape().as_typed_shape() {
        TypedShape::Capsule(capsule) => Collider::capsule(capsule.radius + value, capsule.height()),
        TypedShape::Ball(ball) => Collider::sphere(ball.radius + value),
        _ => todo!("implement the remaining shapes that make sense"),
    }
}
