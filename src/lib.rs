use std::mem;

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

#[derive(Component, Deref)]
pub struct StandingOn(pub Entity);

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

#[derive(Clone, Copy)]
enum SlopePlane {
    Floor,
    Wall,
    Roof,
}

impl SlopePlane {
    fn new(up_direction: Dir3, normal: Dir3, max_floor_angle: f32) -> Self {
        let angle = f32::acos(normal.dot(*up_direction));
        match angle {
            a if a > PI / 2.0 => Self::Roof,
            a if a > max_floor_angle => Self::Wall,
            _ => Self::Floor,
        }
    }

    fn is_floor(&self) -> bool {
        matches!(self, SlopePlane::Floor)
    }

    fn is_wall(&self) -> bool {
        matches!(self, SlopePlane::Wall)
    }

    fn is_roof(&self) -> bool {
        matches!(self, SlopePlane::Roof)
    }
}

pub struct MovementOutput {
    pub position: Vec3,
    pub velocity: Vec3,
    pub remaining: Vec3,
    pub slide_distance: f32,
    pub travel_distance: f32,
    pub floor_entity: Option<Entity>,
    pub is_on_wall: bool,
    pub is_on_roof: bool,
}

// FIXME: moving towards a wall while also climbing up a slope results in sudden bursts of speed.
// This is appears to be caused by both "climb walkable slopes" and "step over walls".
pub fn collide_and_slide3(
    mut position: Vec3,
    rotation: Quat,
    mut velocity: Vec3,
    up_direction: Dir3,
    skin_width: f32,
    max_floor_angle: f32,
    max_step_height: f32,
    min_step_depth: f32,
    floor_snap_length: f32,
    climb_walkable_slopes: bool,
    apply_remaining_motion: bool,
    shape: &Collider,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    delta_time: f32,
    gizmos: &mut Gizmos,
) -> MovementOutput {
    let inflated_shape = inflate_shape(shape, skin_width);

    let mut remaining_motion = velocity * delta_time;

    let mut floor_entity = None;
    let mut is_on_wall = false;
    let mut is_on_roof = false;

    // 0. Detect grounded and snap out of floor.

    enum SlopeIndex {
        First,
        Second { first_normal: Vec3 },
        Third,
    }

    let mut slope_index = SlopeIndex::First;

    for _ in 0..16 {
        let Ok((direction, length)) = Dir3::new_and_length(remaining_motion) else {
            break;
        };

        // 1. Climb walkable slopes.
        // This is done to avoid losing speed when walking up slopes and ledges.
        //
        // TODO: this should be possible to disable via an option.
        if climb_walkable_slopes {
            let step_up = 2.0 * skin_width * up_direction;
            let step_forward = direction * length;

            // Sweep down from the target step location.
            if let Some(hit) = spatial.cast_shape(
                &inflated_shape,
                position + step_up + step_forward,
                rotation,
                -up_direction,
                &ShapeCastConfig {
                    max_distance: skin_width * 2.0,
                    target_distance: skin_width,
                    compute_contact_on_penetration: true,
                    ignore_origin_penetration: true,
                },
                filter,
            ) {
                {
                    // Only climb when hit point is below character, e.g., not roof.
                    if hit.point1.dot(*up_direction) < position.dot(*up_direction) {
                        let slope = SlopePlane::new(
                            up_direction,
                            Dir3::new_unchecked(hit.normal1),
                            max_floor_angle,
                        );
                        if slope.is_floor() {
                            // FIXME: this may make it possible to step through walls.
                            let incoming = -up_direction * hit.distance;

                            position += step_up + step_forward + incoming;
                            remaining_motion -= step_forward;

                            gizmos.sphere(Isometry3d::from_translation(hit.point1), 1., WHITE);
                        } else {
                            gizmos.sphere(Isometry3d::from_translation(hit.point1), 1., RED);
                        }
                    } else {
                        gizmos.sphere(Isometry3d::from_translation(hit.point1), 1., BLACK);
                    }
                };
            }
        }

        // 2. Sweep for collisions in movement direction.
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
            //
            // Sometimes this is undesired, like when using LinearVelocity
            // the rest of the movement would be applied twice since kinematic bodies
            // are automatically moved according to velocity each frame.
            if apply_remaining_motion {
                position += mem::take(&mut remaining_motion);
            }
            break;
        };

        let incoming = direction * (hit.distance - skin_width); // Move a little bit away from the surface.

        // Move to the hit point.
        position += incoming;
        remaining_motion -= incoming;

        let slope = SlopePlane::new(
            up_direction,
            Dir3::new_unchecked(hit.normal1),
            max_floor_angle,
        );

        match slope {
            SlopePlane::Floor => floor_entity = Some(hit.entity),
            SlopePlane::Wall => is_on_wall = true,
            SlopePlane::Roof => is_on_roof = true,
        }

        // 3. Attempt to step over walls.
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
                        if let SlopePlane::Floor = SlopePlane::new(
                            up_direction,
                            Dir3::new_unchecked(hit.normal1),
                            max_floor_angle,
                        ) {
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
                        };
                    }
                }
            }
        }

        // 4. Slide if no step was possible.
        if !did_step {
            match slope_index {
                // We want to remove all motion in the direction of the hit normal,
                // i.e., project the motion on the normal plane.
                SlopeIndex::First => {
                    remaining_motion = remaining_motion.reject_from(hit.normal1);
                    velocity = velocity.reject_from(hit.normal1);

                    slope_index = SlopeIndex::Second {
                        first_normal: hit.normal1,
                    };
                }
                // Since the previous iteration changed the motion direction, doing so again might
                // result in a new motion where no parts of it existed in the original velocity.
                // This may result in the character "shaking" when walking into a corner.
                //
                // We solve this by projecting onto a "crease" normal instead.
                SlopeIndex::Second { first_normal } => {
                    // Do nothing if the normal is identical to the previous one.
                    if first_normal.dot(hit.normal1) < 1.0 - 1e-4 {
                        let crease = first_normal.cross(hit.normal1).normalize();

                        remaining_motion = remaining_motion.project_onto(crease);
                        velocity = velocity.project_onto(crease);

                        slope_index = SlopeIndex::Third;
                    }
                }
                // When the character is sliding on three unique normals, no more motion is possible.
                SlopeIndex::Third => {
                    remaining_motion = Vec3::ZERO;
                    velocity = Vec3::ZERO;
                }
            }
            // Remove vertical motion when the slope is not walkable.
            if !slope.is_floor() {
                remaining_motion -= *up_direction * remaining_motion.dot(*up_direction).max(0.0);
                velocity -= *up_direction * velocity.dot(*up_direction).max(0.0);
            }
        }
    }

    // 5. Snap to floor.
    //
    // FIXME: this can lead to sliding down corners rather than falling when moving off ledges.
    // Unreal engine has an option that fixes this but introduces a new issue if I remember
    // correctly, might be worth looking into.
    if floor_entity.is_none() && floor_snap_length > 0.0 {
        if let Some(hit) = spatial.cast_shape(
            &inflated_shape,
            position,
            rotation,
            -up_direction,
            &ShapeCastConfig {
                max_distance: floor_snap_length,
                target_distance: skin_width,
                compute_contact_on_penetration: true,
                ignore_origin_penetration: true,
            },
            filter,
        ) {
            let slope = SlopePlane::new(
                up_direction,
                Dir3::new_unchecked(hit.normal1),
                max_floor_angle,
            );
            if slope.is_floor() {
                let incoming = -up_direction * hit.distance;
                position += incoming;

                floor_entity = Some(hit.entity);
            }
        }
    }

    // 6. Solve remaining overlaps.
    let mut overlaps = SmallVec::<[_; 8]>::new_const();
    for _ in 0..16 {
        overlaps.clear();
        if let Some(motion) = solve_overlaps(
            &inflated_shape,
            position,
            rotation,
            up_direction,
            max_floor_angle,
            &mut overlaps,
            filter,
            spatial,
        ) {
            let mut slope_index = SlopeIndex::First;
            for overlap in &overlaps {
                if overlap.depth > 1e-4 {
                    // Do the same trick as with sliding but only for velocity.
                    match slope_index {
                        SlopeIndex::First => {
                            velocity = velocity.reject_from(overlap.normal);

                            slope_index = SlopeIndex::Second {
                                first_normal: overlap.normal,
                            };
                        }
                        SlopeIndex::Second { first_normal } => {
                            if overlap.normal.dot(first_normal) < 1.0 - 1e-4 {
                                let crease = first_normal.cross(overlap.normal).normalize();
                                velocity = velocity.project_onto(crease);

                                slope_index = SlopeIndex::Third;
                            }
                        }
                        SlopeIndex::Third => {
                            velocity = Vec3::ZERO;
                            break;
                        }
                    }
                }
            }
            position += motion;
        } else {
            break; // No more overlaps.
        }
    }

    MovementOutput {
        position,
        velocity,
        remaining: remaining_motion,
        slide_distance: 0.0,  // TODO: implement.
        travel_distance: 0.0, // TODO: implement.
        floor_entity,
        is_on_wall,
        is_on_roof,
    }
}

/// All the info required to solve an overlap with the world.
#[derive(Debug, Clone, Copy)]
struct OverlapInfo {
    normal: Vec3,
    depth: f32,
}

fn solve_overlaps<T: Array<Item = OverlapInfo>>(
    inflated_shape: &Collider,
    position: Vec3,
    rotation: Quat,
    up_direction: Dir3,
    max_floor_angle: f32,
    overlaps: &mut SmallVec<T>,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
) -> Option<Vec3> {
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
                    normal: hit.normal1,
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
        let offset_vector = match SlopePlane::new(
            up_direction,
            Dir3::new(overlap.normal).unwrap(),
            max_floor_angle,
        ) {
            SlopePlane::Floor => overlap.normal,
            SlopePlane::Wall | SlopePlane::Roof => {
                let up_amount = up_direction.dot(overlap.normal).max(0.0);
                overlap.normal - up_direction * up_amount
            }
        };

        total_depth += overlap.depth;
        motion += overlap.depth * offset_vector;

        // Make sure the next overlaps doesn't move the character further than neccessary.
        for next_overlap in &mut overlaps[i + 1..] {
            if next_overlap.depth < 1e-4 {
                continue;
            }
            let accounted_for =
                f32::max(0.0, offset_vector.dot(next_overlap.normal) * overlap.depth);
            next_overlap.depth = f32::max(0.0, next_overlap.depth - accounted_for);
        }
    }

    if total_depth < 1e-4 {
        return None;
    }

    Some(motion)
}

/// Grow or shrink a [`Collider`].
pub fn inflate_shape(shape: &Collider, inflate: f32) -> Collider {
    match shape.shape().as_typed_shape() {
        TypedShape::Capsule(capsule) => {
            Collider::capsule(capsule.radius + inflate, capsule.height())
        }
        TypedShape::Ball(ball) => Collider::sphere(ball.radius + inflate),
        _ => todo!("implement the remaining shapes that make sense"),
    }
}
