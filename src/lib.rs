use avian3d::{math::PI, parry::shape::TypedShape, prelude::*};
use bevy::{
    ecs::{component::ComponentId, world::DeferredWorld},
    prelude::*,
};
use smallvec::SmallVec;
use std::mem;

// TODO: components and systems for character.

#[derive(SystemSet, Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct CharacterMovementSystems;

pub struct CharacterMovementPlugin;

impl Plugin for CharacterMovementPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedUpdate,
            update_character_movement
                .before(PhysicsSet::Prepare)
                .in_set(CharacterMovementSystems),
        );
    }
}

#[derive(Component, Reflect, PartialEq)]
pub struct Character {
    pub up_direction: Dir3,
    pub skin_width: f32,
    pub floor_snap_distance: f32,
    pub max_floor_angle: f32,
    pub climb_up_walls: bool,
    pub preserve_speed_on_floor: bool,
    /// The distance the character moved last update
    /// divided by delta time.
    pub last_update_velocity: Vec3,
    pub floor: Option<Floor>,
    pub velocity: Vec3,
}

impl Default for Character {
    fn default() -> Self {
        Self {
            up_direction: Dir3::Y,
            skin_width: 0.1,
            floor_snap_distance: 0.5,
            max_floor_angle: 45_f32.to_radians(),
            climb_up_walls: false,
            preserve_speed_on_floor: true,
            floor: None,
            last_update_velocity: Vec3::ZERO,
            velocity: Vec3::ZERO,
        }
    }
}

fn update_character_movement(
    spatial: SpatialQuery,
    mut gizmos: Gizmos,
    mut query: Query<(
        Entity,
        &mut Character,
        &mut LinearVelocity,
        &Collider,
        &mut Transform,
    )>,
    time: Res<Time>,
) {
    for (entity, mut character, mut _velocity, shape, mut transform) in &mut query {
        let filter = SpatialQueryFilter::from_excluded_entities([entity]);

        let movement = move_and_slide(
            character.floor,
            transform.translation,
            character.velocity,
            transform.rotation,
            MovementConfig {
                up_direction: character.up_direction,
                skin_width: character.skin_width,
                floor_snap_distance: character.floor_snap_distance,
                max_floor_angle: character.max_floor_angle,
                climb_up_walls: character.climb_up_walls,
                max_slide_count: 12,
                preserve_speed_on_floor: character.preserve_speed_on_floor,
                apply_remaining_motion: true,
            },
            shape,
            &spatial,
            &filter,
            time.delta_secs(),
            |_| {},
            &mut gizmos,
        );

        transform.translation = movement.position;
        character.velocity = movement.velocity;
        // character.velocity =
        //     movement.applied_motion / time.delta_secs() + movement.remaining_motion;

        if character.floor.is_none() && movement.floor.is_some() {
            println!("AIR --> GROUND");
        }

        if character.floor.is_some() && movement.floor.is_none() {
            println!("GROUND --> AIR");
        }

        character.floor = movement.floor;

        character.last_update_velocity = movement.applied_motion / time.delta_secs();
    }
}

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

#[derive(Reflect, Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum SlopePlane {
    Floor,
    Wall,
    Roof,
}

impl SlopePlane {
    pub fn new(normal: Dir3, up_direction: Dir3, max_floor_angle: f32) -> Self {
        Self::new_and_angle(normal, up_direction, max_floor_angle).0
    }

    pub fn new_and_angle(normal: Dir3, up_direction: Dir3, max_floor_angle: f32) -> (Self, f32) {
        let angle = f32::acos(normal.dot(*up_direction));

        if angle - 0.01 > PI / 2.0 {
            (Self::Roof, angle)
        } else if angle >= max_floor_angle {
            (Self::Wall, angle)
        } else {
            (Self::Floor, angle)
        }
    }

    pub fn is_floor(&self) -> bool {
        matches!(self, SlopePlane::Floor)
    }

    pub fn is_wall(&self) -> bool {
        matches!(self, SlopePlane::Wall)
    }

    pub fn is_roof(&self) -> bool {
        matches!(self, SlopePlane::Roof)
    }
}

// TODO: this should not be a component
#[derive(Component, Reflect, Debug, PartialEq, Clone, Copy)]
pub struct Floor {
    pub entity: Entity,
    pub normal: Dir3,
}

/// A collision which occured during a call to [`move_and_slide`].
#[derive(Event, Debug)]
pub struct MovementCollision {
    /// The velocity of the character before the collision occured.
    pub velocity: Vec3,
    /// The hit result of the shape cast.
    pub hit: ShapeHitData,
}

/// Configuration for [`move_and_slide`].
pub struct MovementConfig {
    pub up_direction: Dir3,
    pub skin_width: f32,
    pub floor_snap_distance: f32,
    pub max_floor_angle: f32,
    pub climb_up_walls: bool,
    pub preserve_speed_on_floor: bool,
    pub max_slide_count: usize,
    /// If `true` the remaining motion will be applied to the [`MovementOutput::applied_motion`] after the last slide,
    /// unless the slide count exceeded [`max_slide_count`](Self::max_slide_count).
    pub apply_remaining_motion: bool,
}

/// The result of a call to [`move_and_slide`].
pub struct MovementOutput {
    /// The actual movement of the character.
    pub applied_motion: Vec3,
    /// The modified velocity after sliding. This is usually not a good representation
    /// of the actual motion of the character, see [`applied_motion`](Self::applied_motion).
    pub velocity: Vec3,
    /// The final position after applying all the movement motion.
    pub position: Vec3,
    /// The remaining movement motion, this will be [`Vec3::ZERO`] unless [`MovementConfig::apply_remaining_motion`]
    /// was `true` or if the slide count exceeded that of [`MovementConfig::max_slide_count`].
    pub remaining_motion: Vec3,
    /// Contains the [`Entity`] and normal of the last floor the character was standing on, if any.
    /// Note that this can be [`Some`] even if the character was not standing on the floor at the
    /// end of the call to [`move_and_slide`].
    pub floor: Option<Floor>,
    /// The amount the character is still overapping geometry at the end of the update,
    /// including the [`MovementConfig::skin_width`].
    /// This can be used to determine if the character is stuck.
    pub overlap_amount: f32,
}

// FIXME:
// - ~~Jumping doesn't work when walking up steep slopes~~.
// - Hugging wall slopes when climb_up_walls is false results in gravity being canceled out somehow. pls fix
//
// TODO:
// - More configuration options.
// - Stair stepping.
// - Moving platforms (properly this time).
/// Sweep in the direction of `velocity` and slide along surfaces.
///
/// Returns [`MovementOutput`] which contains information about the movement,
/// such as the final position and the modified velocity.
///
/// - `previous_floor`: Used to determine if we should snap to the floor.
/// - `origin`: The started position, usually [`Transform::translation`].
/// - `velocity`: The velocity of the character.
/// - `rotation`: The rotation of the character.
/// - `config`: see [`MovementConfig`].
///
/// ..and so on and so on..
#[must_use]
pub fn move_and_slide(
    previous_floor: Option<Floor>,
    origin: Vec3,
    mut velocity: Vec3,
    rotation: Quat,
    config: MovementConfig,
    shape: &Collider,
    spatial: &SpatialQuery,
    filter: &SpatialQueryFilter,
    delta_secs: f32,
    mut on_collide: impl FnMut(MovementCollision),
    gizmos: &mut Gizmos,
) -> MovementOutput {
    let mut applied_motion = Vec3::ZERO;
    let mut remaining_motion = velocity * delta_secs;

    let inflated_shape = inflate_shape(shape, config.skin_width); // Make the shape thick.

    let mut floor = None;

    let mut is_on_wall = false;

    let mut overlaps_buffer = Vec::with_capacity(8);

    for _ in 0..config.max_slide_count {
        // Slide untill there's no more motion.
        let Ok((direction, length)) = Dir3::new_and_length(remaining_motion) else {
            break;
        };

        // 1. Sweep in the movement direction.
        let Some(hit) = spatial.cast_shape(
            shape,
            origin + applied_motion,
            rotation,
            direction,
            &ShapeCastConfig {
                max_distance: length + config.skin_width,
                target_distance: config.skin_width,
                compute_contact_on_penetration: true,
                ignore_origin_penetration: true,
            },
            filter,
        ) else {
            // If nothing was hit, apply the remaining motion.
            if config.apply_remaining_motion {
                applied_motion += mem::take(&mut remaining_motion);
            }

            break;
        };

        let incoming = direction * (hit.distance - config.skin_width);

        // Move as close to the geometry as possible.
        applied_motion += incoming;
        remaining_motion -= incoming;

        on_collide(MovementCollision { velocity, hit }); // Trigger callbacks.

        let normal = Dir3::new_unchecked(hit.normal1);
        let slope = SlopePlane::new(normal, config.up_direction, config.max_floor_angle);

        let is_stuck = hit.distance == 0.0;

        // 2. Slide along slopes and push the character up.
        if slope.is_floor() && !is_stuck {
            floor = Some(Floor {
                entity: hit.entity,
                normal,
            });

            // Slide on floor.
            remaining_motion = project_on_floor(
                remaining_motion,
                normal,
                config.up_direction,
                config.preserve_speed_on_floor,
            );
        } else {
            let valid_slope: fn(SlopePlane) -> bool = match is_stuck {
                true => |_| true,                  // If the character is stuck, fix all overlaps.
                false => |slope| slope.is_floor(), // Otherwise, only fix overlaps with the floor.
            };

            // Attempt to push the character away from the floor.
            // This is only neccessary when the hit normal was not a floor
            // to avoid randomly stopping at small obstacles.
            if let Some(fixup) = get_penetration(
                &mut overlaps_buffer,
                &inflated_shape,
                origin + applied_motion,
                rotation,
                -config.up_direction,
                filter,
                spatial,
                |overlap| {
                    valid_slope(SlopePlane::new(
                        config.up_direction,
                        Dir3::new_unchecked(overlap.hit.normal1),
                        config.max_floor_angle,
                    ))
                },
            ) {
                applied_motion += fixup;

                let normal = overlaps_buffer[1..].iter().map(|o| o.hit.normal1).fold(
                    Dir3::new_unchecked(overlaps_buffer[0].hit.normal1),
                    |acc, inc| match inc.dot(*config.up_direction) >= acc.dot(*config.up_direction)
                    {
                        true => Dir3::new_unchecked(inc),
                        false => acc,
                    },
                );

                remaining_motion = project_on_floor(
                    remaining_motion,
                    normal,
                    config.up_direction,
                    config.preserve_speed_on_floor,
                );
            } else {
                // If there's no penetration with the floor then it's probably a wall, right?

                is_on_wall = true;

                // Slide motion and velocity.
                remaining_motion = project_on_wall(
                    remaining_motion,
                    normal,
                    config.up_direction,
                    config.climb_up_walls,
                );
                velocity =
                    project_on_wall(velocity, normal, config.up_direction, config.climb_up_walls);
            };
        }
    }

    // 3. Solve remaining overlaps.
    let mut overlap_amount = 0.0;

    for _ in 0..1 {
        overlap_amount = 0.0;

        if let Some(..) = get_penetration(
            &mut overlaps_buffer,
            &inflated_shape,
            origin + applied_motion,
            rotation,
            -config.up_direction,
            filter,
            spatial,
            |_| true,
        ) {
            // Splitting the penetration fix into componentes are important to make sure
            // the character doesn't get pushed up or to the side when it's not supposed to.
            let mut vertical_fix = Vec3::ZERO;
            let mut horizontal_fix = Vec3::ZERO;
            let mut fix = Vec3::ZERO;

            for overlap in &overlaps_buffer {
                // The overlap amount can be used to check if the character is stuck.
                overlap_amount += overlap.depth;

                let normal = Dir3::new_unchecked(overlap.hit.normal1);
                let slope = SlopePlane::new(normal, config.up_direction, config.max_floor_angle);

                // Trigger callbacks.
                on_collide(MovementCollision {
                    velocity,
                    hit: overlap.hit,
                });

                match slope {
                    // Only push the character on the horizontal axis.
                    SlopePlane::Floor => {
                        vertical_fix += overlap.penetration();

                        // We may still be overlapping a lil bit after jumping.
                        if previous_floor.is_some() {
                            floor = Some(Floor {
                                entity: overlap.hit.entity,
                                normal,
                            });
                        }
                    }
                    // TODO: rooof might require special behavoir, idk.
                    SlopePlane::Wall | SlopePlane::Roof => {
                        is_on_wall = true; // A roof is just a different type of wall.

                        velocity = project_on_wall(
                            velocity,
                            normal,
                            config.up_direction,
                            config.climb_up_walls,
                        );

                        match config.climb_up_walls {
                            true => fix += overlap.penetration(),
                            false => horizontal_fix += overlap.penetration(),
                        }
                    }
                }
            }

            horizontal_fix = horizontal_fix.reject_from_normalized(*config.up_direction);
            vertical_fix = vertical_fix.project_onto_normalized(*config.up_direction);

            applied_motion += fix + horizontal_fix + vertical_fix;
        } else {
            break;
        }
    }

    // 4. Detect and snap to the floor.
    //
    // We skip this when there was no floor previously to avoid suddenly
    // snapping to the ground when falling off a ledge or after jumping.
    if previous_floor.is_some() && floor.is_none() {
        let aabb = shape.aabb(Vec3::ZERO, rotation);
        let half_height = aabb.size().dot(*config.up_direction) / 2.0;
        let ray_length = half_height + config.skin_width + config.floor_snap_distance + 0.01;

        spatial.shape_hits_callback(
            &inflated_shape,
            origin + applied_motion,
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
                let normal = Dir3::new_unchecked(hit.normal1);
                let slope = SlopePlane::new(normal, config.up_direction, config.max_floor_angle);

                if !slope.is_floor() {
                    return true;
                }

                floor = Some(Floor {
                    entity: hit.entity,
                    normal,
                });

                // Don't snap when climbing up a wall.
                if config.climb_up_walls && is_on_wall && config.up_direction.dot(velocity) > 0.0 {
                    return false;
                }

                // Temporary hack to make sure the floor is not blocked by a wall or something.
                let Some(..) = spatial.cast_ray(
                    origin + applied_motion,
                    Dir3::new(hit.point1 - (origin + applied_motion)).unwrap(),
                    ray_length,
                    false,
                    filter,
                ) else {
                    return true;
                };

                if hit.distance <= config.floor_snap_distance {
                    applied_motion -= config.up_direction * hit.distance; // Snap to the floor.
                }

                false
            },
        );
    }

    MovementOutput {
        applied_motion,
        velocity,
        position: origin + applied_motion,
        remaining_motion,
        floor,
        overlap_amount,
    }
}

/// Project the `vector` onto the `normal` plane.
///
/// If `allow_sliding_up` is `false`, the horizontal part of the `vector` (relative to `up_direction`)
/// will not contribute to the positive vertical part of the resulting vector.
#[must_use]
pub fn project_on_wall(
    vector: Vec3,
    normal: Dir3,
    up_direction: Dir3,
    allow_sliding_up: bool,
) -> Vec3 {
    match allow_sliding_up {
        true => vector.reject_from_normalized(*normal),
        false => {
            let DecomposedMotion {
                mut vertical,
                mut horizontal,
            } = DecomposedMotion::new(vector, up_direction);

            vertical = vertical.reject_from_normalized(*normal);

            horizontal = horizontal.reject_from_normalized(*normal);
            horizontal -= horizontal.dot(*up_direction).max(0.0) * up_direction;

            vertical + horizontal
        }
    }
}

/// Project `vector` onto the `normal` plane while keeping the original horizontal orientation of the `vector`.
///
/// When `preserve_speed` is `true` the magnitude of the resulting vector will remain the same as that of the input `vector`.
#[must_use]
pub fn project_on_floor(
    vector: Vec3,
    normal: Dir3,
    up_direction: Dir3,
    preserve_speed: bool,
) -> Vec3 {
    let horizontal_motion = vector.reject_from_normalized(*up_direction);

    let horizontal_length_sq = horizontal_motion.length_squared();

    if horizontal_length_sq < 1e-4 {
        return Vec3::ZERO;
    }

    let Ok(tangent) = Dir3::new(horizontal_motion.cross(*up_direction)) else {
        return horizontal_motion;
    };

    let Ok(direction) = Dir3::new(normal.cross(*tangent)) else {
        todo!("dhaowidh") // I don't know if this can even fail???
    };

    let horizontal_motion = horizontal_motion.project_onto_normalized(*direction);

    // Scale the projected vector with the old length to preserve the magnitude.
    if preserve_speed {
        let Ok(horizontal_direction) = Dir3::new(horizontal_motion) else {
            return Vec3::ZERO;
        };
        f32::sqrt(horizontal_length_sq) * horizontal_direction
    } else {
        horizontal_motion
    }
}

struct DecomposedMotion {
    vertical: Vec3,
    horizontal: Vec3,
}

impl DecomposedMotion {
    /// Decompose the vertical and horizontal part of `motion`, relative to `up_direction`.
    fn new(motion: Vec3, up_direction: Dir3) -> Self {
        let vertical = motion.project_onto(*up_direction);
        let horizontal = motion - vertical;
        Self {
            vertical,
            horizontal,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct OverlapInfo {
    depth: f32,
    hit: ShapeHitData,
}

impl OverlapInfo {
    fn penetration(&self) -> Vec3 {
        self.depth * self.hit.normal1
    }
}

/// Push [`OverlapInfo`] with all bodies penetrating `shape` and `condition` returns true, untill `overlaps` reaches it's capacity.
///
/// `overlaps` will be cleared at the start of the function.
#[must_use]
fn get_penetration(
    overlaps: &mut Vec<OverlapInfo>,
    shape: &Collider,
    position: Vec3,
    rotation: Quat,
    direction: Dir3,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    mut condition: impl FnMut(&OverlapInfo) -> bool,
) -> Option<Vec3> {
    overlaps.clear();

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
                depth: depth_sq.sqrt(),
                hit,
            };
            if depth_sq > 1e-8 && condition(&overlap) {
                overlaps.push(overlap);
            }
            overlaps.capacity() > overlaps.len()
        },
    );

    if overlaps.is_empty() {
        return None;
    }

    // Larger overlaps are more important, so they go first in the queue.
    overlaps.sort_by(|a, b| b.depth.total_cmp(&a.depth));

    let mut fixup = Vec3::ZERO;
    let mut total_depth = 0.0;

    // Solve each overlap.
    for i in 0..overlaps.len() {
        let overlap = overlaps[i];

        fixup += overlap.depth * overlap.hit.normal1; // Accumulate fixup.
        total_depth += overlap.depth;

        // Make sure the next overlaps doesn't move the character further than neccessary.
        for next_overlap in &mut overlaps[i + 1..] {
            if next_overlap.depth < 1e-4 {
                continue;
            }
            let accounted_for = f32::max(
                0.0,
                overlap.hit.normal1.dot(next_overlap.hit.normal1) * overlap.depth,
            );
            next_overlap.depth = f32::max(0.0, next_overlap.depth - accounted_for);
        }
    }

    if total_depth < 1e-4 {
        return None;
    }

    Some(fixup)
}

/// Grow or shrink a [`Collider`].
pub fn inflate_shape(shape: &Collider, value: f32) -> Collider {
    match shape.shape().as_typed_shape() {
        TypedShape::Capsule(capsule) => Collider::capsule(capsule.radius + value, capsule.height()),
        TypedShape::Ball(ball) => Collider::sphere(ball.radius + value),
        _ => todo!("implement the remaining shapes that make sense"),
    }
}
