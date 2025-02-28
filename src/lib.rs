use avian3d::{math::PI, parry::shape::TypedShape, prelude::*};
use bevy::{color::palettes::css::*, prelude::*};
use std::{fmt::Debug, mem};

pub mod seegull;

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

/// Collisions that occured during the last movement.
#[derive(Component, Default, Deref)]
pub struct MovementCollisions(Vec<MovementCollision>);

#[derive(Component, Default)]
#[require(MovementConfig, MovementCollisions, RigidBody(|| RigidBody::Kinematic))]
pub struct CharacterBody {
    /// The desired movement acceleration.
    ///
    /// This is projected on slopes and applied to the character [`velocity`](Self::velocity)
    /// before being reset every movement update.
    ///
    /// Because this is projected on slopes, it's ideal for movement input. For forces such
    /// as gravity, it's better to modify [`velocity`](Self::velocity) directly.
    pub acceleration: Vec3,
    /// The current velocity of the character. The character will move by this amount,
    /// scaled by [`Time::delta_secs`] very movement update.
    pub velocity: Vec3,
    /// The last movement of the character, divided by [`Time::delta_secs`].
    pub last_update_velocity: Vec3,
    /// Contains information about the floor the character is standing on, if any.
    pub floor: Option<Floor>,
}

fn update_character_movement(
    spatial: SpatialQuery,
    mut gizmos: Gizmos,
    mut query: Query<(
        Entity,
        &mut CharacterBody,
        &mut MovementCollisions,
        &MovementConfig,
        &Collider,
        &mut Transform,
    )>,
    time: Res<Time>,
) {
    for (entity, mut character, mut collisions, config, shape, mut transform) in &mut query {
        let acceleration = mem::take(&mut character.acceleration);

        if let Some(floor) = character.floor {
            if floor.normal.dot(acceleration) > 0.0 {
                character.velocity += slide_on_floor(
                    acceleration,
                    floor.normal,
                    config.up_direction,
                    config.preserve_speed_on_floor,
                );
            } else {
                character.velocity += acceleration;
            }
        } else if let Some(wall) = collisions.last() {
            character.velocity +=
                slide_on_wall(acceleration, wall.normal, config.up_direction, false);
        } else {
            character.velocity += acceleration;
        }

        collisions.0.clear();

        let filter = SpatialQueryFilter::from_excluded_entities([entity]);

        let movement = move_and_slide(
            character.floor,
            transform.translation,
            character.velocity,
            transform.rotation,
            config.clone(),
            shape,
            &spatial,
            &filter,
            time.delta_secs(),
            |collision| {
                collisions.0.push(collision);
            },
            &mut gizmos,
        );

        transform.translation = movement.position;
        character.velocity = movement.velocity;

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

#[derive(Reflect, Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum Slope {
    Floor,
    Wall,
    Roof,
}

impl Slope {
    #[track_caller]
    pub fn new(normal: Dir3, up_direction: Dir3, max_floor_angle: f32) -> Self {
        Self::new_and_angle(normal, up_direction, max_floor_angle).0
    }

    #[track_caller]
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
        *self == Self::Floor
    }

    pub fn is_wall(&self) -> bool {
        *self == Self::Wall
    }

    pub fn is_roof(&self) -> bool {
        *self == Self::Roof
    }
}

/// A collision which occured during a call to [`move_and_slide`].
#[derive(Event, Debug)]
pub struct MovementCollision {
    /// The velocity of the character before the collision occured.
    pub velocity: Vec3,
    /// The entity the character interacted with.
    pub entity: Entity,
    pub point: Vec3,
    pub normal: Dir3,
    pub slope: Slope,
}

impl MovementCollision {
    fn new(velocity: Vec3, slope: Slope, hit: ShapeHitData) -> Self {
        Self {
            velocity,
            entity: hit.entity,
            point: hit.point1,
            normal: Dir3::new_unchecked(hit.normal1),
            slope,
        }
    }
}

#[derive(Reflect, Debug, PartialEq, Clone, Copy)]
pub struct Floor {
    pub entity: Entity,
    pub normal: Dir3,
}

/// Configuration for [`move_and_slide`].
#[derive(Component, Reflect, PartialEq, Clone)]
pub struct MovementConfig {
    pub up_direction: Dir3,
    pub skin_width: f32,
    pub floor_snap_distance: f32,
    pub max_floor_angle: f32,
    /// The maximum height the character can step when hitting a wall.
    pub max_step_height: f32,
    /// The maximum forward distance the character can step when hitting a wall.
    pub max_step_depth: f32,
    /// If `true`, the character will be allowed to walk from the floor and onto slopes
    /// that are too steep to otherwise walk on.
    ///
    /// **Note**: This currently does not work properly when [`floor_snap_distance`](Self::floor_snap_distance) is non zero.
    pub climb_up_walls: bool,
    /// If `true`, moving up slopes won't affect the speed of the character.
    pub preserve_speed_on_floor: bool,
    pub slide_iterations: usize,
    /// The maximum number of times we try to seperate the body from the environemtn.
    /// It's recomended with at least 1 but greater values has an impact on performance.
    pub depenetrate_iterations: usize,
    /// The maximum number of times we try to find an optimal step position
    /// when hitting a wall.
    pub step_iterations: usize,
}

impl Default for MovementConfig {
    fn default() -> Self {
        Self {
            up_direction: Dir3::Y,
            skin_width: 0.2,
            floor_snap_distance: 1.0,
            max_floor_angle: 45_f32.to_radians(),
            max_step_height: 1.0,
            max_step_depth: 1.0,
            climb_up_walls: false,
            preserve_speed_on_floor: true,
            slide_iterations: 12,
            depenetrate_iterations: 6,
            step_iterations: 6,
        }
    }
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

#[must_use]
fn move_and_collide(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    length: f32,
    skin_width: f32,
    spatial: &SpatialQuery,
    filter: &SpatialQueryFilter,
) -> Option<(Vec3, ShapeHitData)> {
    let hit = spatial.cast_shape(
        shape,
        origin,
        rotation,
        direction,
        &ShapeCastConfig {
            max_distance: length + skin_width,
            target_distance: skin_width,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: true,
        },
        filter,
    )?;

    Some((direction * (hit.distance - skin_width), hit))
}

// FIXME:
// - ~~Jumping doesn't work when walking up steep slopes~~.
// - ~~Hugging wall slopes when climb_up_walls is false results in gravity being canceled out somehow. pls fix~~
// - Floor snap is making climb up walls not work while previously on the floor.
//
// TODO:
// - More configuration options.
// - Stair stepping.
// - Moving platforms (properly this time).
// - gravity pass?
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
/// ..and so on and so on
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
    let original_direction = Dir3::new(velocity).ok();

    let mut applied_motion = Vec3::ZERO;
    let mut remaining_motion = velocity * delta_secs;

    let inflated_shape = inflate_shape(shape, config.skin_width); // Make the shape thick.

    let mut floor = None;

    let mut is_on_wall = false;

    let mut overlaps_buffer = Vec::with_capacity(8);

    // while remaining_motion.length_squared() > 1e-4 {}

    for _ in 0..config.slide_iterations {
        let Ok((direction, length)) = Dir3::new_and_length(remaining_motion) else {
            break;
        };

        // 1. Sweep in the movement direction.
        let Some((incoming, hit)) = move_and_collide(
            shape,
            origin + applied_motion,
            rotation,
            direction,
            length,
            config.skin_width,
            spatial,
            filter,
        ) else {
            // If nothing was hit, apply the remaining motion.
            applied_motion += mem::take(&mut remaining_motion);
            break;
        };

        // Move as close to the geometry as possible.
        applied_motion += incoming;
        remaining_motion -= incoming;

        let normal = Dir3::new_unchecked(hit.normal1);
        let slope = Slope::new(normal, config.up_direction, config.max_floor_angle);

        on_collide(MovementCollision::new(velocity, slope, hit)); // Trigger callbacks.

        let is_stuck = hit.distance == 0.0;

        // 2. Slide along slopes and push the character up.
        if slope.is_floor() && !is_stuck {
            floor = Some(Floor {
                entity: hit.entity,
                normal,
            });

            // Slide on floor.
            remaining_motion = slide_on_floor(
                remaining_motion,
                normal,
                config.up_direction,
                config.preserve_speed_on_floor,
            );
        } else {
            let validate_slope: fn(Slope) -> bool = match is_stuck {
                true => |_| true,                  // If the character is stuck, fix all overlaps.
                false => |slope| slope.is_floor(), // Otherwise, only fix overlaps with the floor.
            };

            // Attempt to push the character up and away from the floor when we hit a wall.
            if let Some(fixup) = get_penetration(
                &mut overlaps_buffer,
                &inflated_shape,
                origin + applied_motion,
                rotation,
                -config.up_direction,
                filter,
                spatial,
                |overlap| {
                    validate_slope(Slope::new(
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

                remaining_motion = slide_on_floor(
                    remaining_motion,
                    normal,
                    config.up_direction,
                    config.preserve_speed_on_floor,
                );
            } else {
                // Attempt to step up walls.
                if previous_floor.is_some() {
                    if let Ok(direction) =
                        Dir3::new(applied_motion.reject_from_normalized(*config.up_direction))
                    {
                        if let Some((vertical, depth)) = step_up_wall(
                            &inflated_shape,
                            origin + applied_motion,
                            rotation,
                            direction,
                            config.up_direction,
                            config.skin_width,
                            config.max_step_height,
                            config.max_step_depth,
                            config.max_floor_angle,
                            config.step_iterations,
                            spatial,
                            filter,
                        ) {
                            let horizontal = direction * depth;
                            applied_motion += vertical + horizontal;

                            if let Ok((direction, length)) = Dir3::new_and_length(remaining_motion)
                            {
                                remaining_motion -= direction * f32::min(length, depth);
                            }

                            continue; // Saved from the wall.
                        }
                    }
                }

                // Unable to step, we accept that we hit a wall (or roof I guess).
                is_on_wall = true;

                // Slide remaining motion and velocity on the wall.
                remaining_motion = slide_on_wall(
                    remaining_motion,
                    normal,
                    config.up_direction,
                    previous_floor.is_none() || config.climb_up_walls,
                );

                // See Quake2: "If velocity is against original velocity, stop ead to avoid tiny oscilations in sloping corners."
                if original_direction
                    .map(|original_direction| remaining_motion.dot(*original_direction) <= 0.0)
                    .unwrap_or(true)
                {
                    break;
                }

                velocity = slide_on_wall(
                    velocity,
                    normal,
                    config.up_direction,
                    previous_floor.is_none() || config.climb_up_walls,
                );
            };
        }
    }

    // 3. Solve overlaps.
    let mut overlap_amount = 0.0;

    for _ in 0..config.depenetrate_iterations {
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

            // If we were on the floor previously, we try stepping in the movement direction
            // if we hit a wall.
            let step_direction = previous_floor.and_then(|_| {
                Dir3::new(applied_motion.reject_from_normalized(*config.up_direction)).ok()
            });

            for overlap in &overlaps_buffer {
                // The overlap amount can be used to check if the character is stuck.
                overlap_amount += overlap.depth;

                let normal = Dir3::new_unchecked(overlap.hit.normal1);
                let slope = Slope::new(normal, config.up_direction, config.max_floor_angle);

                // Trigger callbacks.
                on_collide(MovementCollision::new(velocity, slope, overlap.hit));

                match slope {
                    // Only push the character on the horizontal axis.
                    Slope::Floor => {
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
                    Slope::Wall | Slope::Roof => {
                        // Try stepping over obstacles.
                        if let Some(direction) = step_direction {
                            if let Some((vertical, depth)) = step_up_wall(
                                &inflated_shape,
                                origin + applied_motion,
                                rotation,
                                direction,
                                config.up_direction,
                                config.skin_width,
                                config.max_step_height,
                                config.max_step_depth,
                                config.max_floor_angle,
                                config.step_iterations,
                                spatial,
                                filter,
                            ) {
                                let horizontal = direction * depth;
                                applied_motion += vertical + horizontal;

                                break; // There's probably no more collisions if we stepped over them.
                            }
                        }

                        is_on_wall = true; // A roof is just a special type of wall.

                        velocity = slide_on_wall(
                            velocity,
                            normal,
                            config.up_direction,
                            previous_floor.is_none() || config.climb_up_walls,
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
        if let Some((incoming, hit)) = move_and_collide(
            shape,
            origin + applied_motion,
            rotation,
            -config.up_direction,
            config.floor_snap_distance,
            config.skin_width,
            spatial,
            filter,
        ) {}

        if let Some((new_floor, offset)) = snap_to_floor(
            shape,
            origin,
            rotation,
            config.up_direction,
            config.skin_width,
            config.floor_snap_distance,
            config.max_floor_angle,
            is_on_wall && config.climb_up_walls,
            spatial,
            filter,
        ) {
            floor = Some(new_floor);
            applied_motion += offset;
        }
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

fn snap_to_floor(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    up_direction: Dir3,
    skin_width: f32,
    floor_snap_distance: f32,
    max_floor_angle: f32,
    is_climbing: bool,
    spatial: &SpatialQuery,
    filter: &SpatialQueryFilter,
) -> Option<(Floor, Vec3)> {
    let min_distance = 0.1;

    let hit = spatial.cast_shape(
        shape,
        origin,
        rotation,
        -up_direction,
        &ShapeCastConfig {
            max_distance: f32::max(min_distance, floor_snap_distance) + skin_width,
            target_distance: skin_width,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: true,
        },
        filter,
    )?;

    let normal = Dir3::new_unchecked(hit.normal1);
    let slope = Slope::new(normal, up_direction, max_floor_angle);

    let mut floor = None;

    if slope.is_floor() {
        floor = Some(Floor {
            entity: hit.entity,
            normal,
        });

        if floor_snap_distance == 0.0 {
            return floor.map(|f| (f, Vec3::ZERO));
        }
    }

    let max_distance = match is_climbing {
        true => min_distance,
        false => f32::max(min_distance, floor_snap_distance),
    };

    let mut offset = Vec3::ZERO;

    spatial.shape_hits_callback(
        &inflate_shape(shape, skin_width),
        origin,
        rotation,
        -up_direction,
        &ShapeCastConfig {
            max_distance,
            target_distance: 0.0,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: true,
        },
        filter,
        |hit| {
            let normal = Dir3::new_unchecked(hit.normal1);
            let slope = Slope::new(normal, up_direction, max_floor_angle);

            if !slope.is_floor() {
                return true; // Continue checking.
            }

            floor = Some(Floor {
                entity: hit.entity,
                normal,
            });

            if hit.distance > floor_snap_distance {
                return true; // Continue checking.
            }

            offset = -hit.distance * up_direction; // Snap to floor.

            false // Stop checking once we found steppable floor.
        },
    );

    floor.map(|f| (f, offset))
}

fn step_up_wall(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    up_direction: Dir3,
    min_height: f32,
    max_height: f32,
    max_depth: f32,
    max_floor_angle: f32,
    iterations: usize,
    spatial: &SpatialQuery,
    filter: &SpatialQueryFilter,
) -> Option<(Vec3, f32)> {
    let mut closest_floor = None;
    let mut wall_depth = 0.0; // Start at the character location.

    let step_size = max_depth / iterations as f32;

    for _ in 0..iterations {
        let depth = match closest_floor {
            Some((_, floor_depth)) => {
                if f32::abs(wall_depth - floor_depth) < 0.01 {
                    break;
                }
                (wall_depth + floor_depth) / 2.0 // Step halfway back.
            }
            None => wall_depth + step_size, // Step forward a little bit.
        };

        if depth > max_depth {
            break;
        }

        let step_forward = depth * direction;
        let step_up = max_height * up_direction;

        let Some(hit) = spatial.cast_shape(
            shape,
            origin + step_forward + step_up,
            rotation,
            -up_direction,
            &ShapeCastConfig {
                max_distance: max_height,
                target_distance: 0.0,
                compute_contact_on_penetration: false,
                ignore_origin_penetration: false,
            },
            filter,
        ) else {
            break; // If there's no hit, then we probably stepped too far.
        };

        // We can't stand here.
        if hit.distance == 0.0 || max_height - hit.distance < min_height {
            break;
        }

        // Subtract a small amount from max_floor_angle to make sure floor snapping still works.
        let slope = Slope::new(
            Dir3::new_unchecked(hit.normal1),
            up_direction,
            max_floor_angle - 0.05,
        );

        if slope.is_floor() {
            let incoming = -hit.distance * up_direction;
            closest_floor = Some((step_up + incoming, depth));
        } else {
            wall_depth = depth;
        }
    }

    closest_floor
}

/// Project the `vector` onto the `normal` plane.
#[must_use]
pub fn slide_on_wall(vector: Vec3, normal: Dir3, up_direction: Dir3, climb_up: bool) -> Vec3 {
    match climb_up {
        true => vector.reject_from_normalized(*normal),
        false => {
            let DecomposedMotion {
                mut vertical,
                mut horizontal,
            } = DecomposedMotion::new(vector, up_direction);

            vertical = vertical.reject_from_normalized(*normal);

            let Ok(tangent) = Dir3::new(normal.cross(*up_direction)) else {
                return vertical + horizontal; // This is not a wall.
            };

            let par_normal = tangent.cross(*up_direction);

            horizontal -= horizontal.dot(par_normal).max(0.0) * par_normal;
            horizontal = horizontal.reject_from_normalized(*normal);

            vertical + horizontal
        }
    }
}

/// Project `vector` onto the `normal` plane while keeping the original horizontal orientation of the `vector`.
///
/// When `preserve_speed` is `true` the magnitude of the resulting vector will remain the same as that of the input `vector`.
#[must_use]
pub fn slide_on_floor(
    vector: Vec3,
    normal: Dir3,
    up_direction: Dir3,
    preserve_speed: bool,
) -> Vec3 {
    let DecomposedMotion {
        mut vertical,
        horizontal,
    } = DecomposedMotion::new(vector, up_direction);

    // Remove vertical velocity going downwards.
    if vertical.dot(*up_direction) < 0.0 {
        vertical = Vec3::ZERO;
    }

    let horizontal_length_sq = horizontal.length_squared();

    if horizontal_length_sq < 1e-4 {
        return vertical;
    }

    let Ok(tangent) = Dir3::new(horizontal.cross(*up_direction)) else {
        return vertical; // Horizontal velocity is zero.
    };

    let Ok(direction) = Dir3::new(normal.cross(*tangent)) else {
        return vertical + horizontal; // Horizontal direction is perfectly perpendicular with the normal.
    };

    let horizontal = horizontal.project_onto_normalized(*direction);

    // Scale the projected vector with the old length to preserve the magnitude.
    if preserve_speed {
        let Ok(horizontal_direction) = Dir3::new(horizontal) else {
            return vertical;
        };
        vertical + f32::sqrt(horizontal_length_sq) * horizontal_direction
    } else {
        vertical + horizontal
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
