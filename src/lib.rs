use std::mem;

use avian3d::{
    math::PI,
    parry::{self, shape::TypedShape},
    prelude::*,
};
use bevy::{
    color::palettes::css::{BLACK, BLUE, GREEN, LIGHT_BLUE, RED, WHITE},
    math::{InvalidDirectionError, VectorSpace},
    prelude::*,
};
use smallvec::{Array, SmallVec};

pub const HALF_PI: f32 = 1.618033988749894848204586834365638118_f32;

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
            a if a > HALF_PI => Self::Roof,
            a if a > max_floor_angle => Self::Wall,
            _ => Self::Floor,
        }
    }

    fn is_floor(&self) -> bool {
        match self {
            SlopePlane::Floor => true,
            SlopePlane::Wall => false,
            SlopePlane::Roof => false,
        }
    }
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

pub enum CharacterValue {
    Absolute(f32),
    Relative(f32),
}

impl CharacterValue {
    pub const ZERO: Self = Self::Absolute(0.0);

    pub fn eval(&self, shape: &Collider) -> f32 {
        match self {
            CharacterValue::Absolute(value) => *value,
            CharacterValue::Relative(value) => {
                let extents: Vec3 = shape.shape().compute_local_aabb().extents().into();
                value * extents.max_element()
            }
        }
    }
}

pub struct CollideAndSlideConfig {
    pub up: Dir3,
    pub max_slope_angle: f32,
    pub skin_width: CharacterValue,
    pub floor_snap_distance: CharacterValue,
    pub normal_nudge: f32, // TODO: don't think this is needed.
    pub max_slide_count: u32,
    pub max_step_height: CharacterValue,
    pub min_step_width: CharacterValue,
    /// Applies the remaining motion to the motion vector when no collision was made.
    /// This should be `false` when using [`LinearVelocity`] to avoid moving at double speed.
    pub apply_remaining_motion: bool,
}

impl Default for CollideAndSlideConfig {
    fn default() -> Self {
        Self {
            up: Dir3::Y,
            max_slope_angle: PI / 2.0,
            skin_width: CharacterValue::Relative(0.01),
            floor_snap_distance: CharacterValue::Relative(0.1),
            normal_nudge: 1e-4,
            max_slide_count: 5,
            apply_remaining_motion: false,
            max_step_height: CharacterValue::Relative(10.0),
            min_step_width: CharacterValue::Relative(0.5),
        }
    }
}

pub struct CollideAndSlideOutput {
    pub motion: Vec3,
    pub remaining: Vec3,
}

pub struct MovementOutput {
    pub position: Vec3,
    pub velocity: Vec3,
    pub remaining: Vec3,
    pub slide_distance: f32,
    pub travel_distance: f32,
}

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
    shape: &Collider,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    delta_time: f32,
    gizmos: &mut Gizmos,
) -> MovementOutput {
    let mut remaining_motion = velocity * delta_time;

    let mut is_on_floor = false;
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
        // 1. Attempt to step over things.
        let (horizontal_dir, horizontal_len) =
            Dir3::new_and_length(remaining_motion.reject_from(*up_direction))
                // Dir3::new_and_length(remaining_motion)
                .map(|(d, l)| (*d, l))
                .unwrap_or_default();

        let aabb = shape.aabb(position, rotation);
        let size = aabb.size();
        let half_height = up_direction.dot(size) / 2.0 + skin_width;

        // if let Ok(horizontal_dir) = Dir3::new(remaining_motion.reject_from(*up_direction))
        {
            let step_up = up_direction * half_height;
            let step_forward = horizontal_dir * horizontal_len;

            // Sweep down from the target step location.
            if let Some(hit) = spatial.cast_shape(
                &inflate_shape(shape, skin_width),
                position + step_up + step_forward,
                rotation,
                -up_direction,
                &ShapeCastConfig {
                    max_distance: half_height,
                    target_distance: skin_width,
                    compute_contact_on_penetration: true,
                    ignore_origin_penetration: true,
                },
                filter,
            ) {
                {
                    let start = rotation * hit.point2 + position;

                    gizmos.sphere(Isometry3d::from_translation(start), 1., WHITE);

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

                        gizmos.sphere(Isometry3d::from_translation(hit.point1), 1., BLACK);
                    } else {
                        gizmos.sphere(Isometry3d::from_translation(hit.point1), 1., RED);
                    }
                };
            }
        }

        let Ok((direction, length)) = Dir3::new_and_length(remaining_motion) else {
            break;
        };

        // Sweep in movement direction.
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
            position += mem::take(&mut remaining_motion);
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
            SlopePlane::Floor => is_on_floor = true,
            SlopePlane::Wall => is_on_wall = true,
            SlopePlane::Roof => is_on_roof = true,
        }

        // 1. Attempt to step over walls.
        let mut did_step = false;
        if max_step_height > 0.0 && matches!(slope, SlopePlane::Wall | SlopePlane::Roof) {
            // if max_step_height > 0.0 || slope.is_floor() {
            if let Ok(horizontal_dir) = Dir3::new(remaining_motion.reject_from(*up_direction)) {
                let horizontal_len = horizontal_dir.dot(remaining_motion);

                let step_up = up_direction * max_step_height;

                let step_forward_dist = min_step_depth + skin_width;
                let step_forward = horizontal_dir * step_forward_dist;
                // let step_forward = horizontal_dir
                //     * (f32::min(min_step_depth, horizontal_dir.dot(remaining_motion)) + skin_width);

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
                    // Only step if the normal is walkable.
                    if let SlopePlane::Floor = SlopePlane::new(
                        up_direction,
                        Dir3::new_unchecked(hit.normal1),
                        max_floor_angle,
                    ) {
                        let incoming = -up_direction * (hit.distance - skin_width);
                        // position += step_up + step_forward + incoming;
                        let new_position = position + step_up + step_forward + incoming;

                        // Place body on the ground after stepping.
                        // position = new_position;
                        // continue;
                        if let Some(hit) = spatial.cast_shape(
                            &inflate_shape(shape, skin_width),
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
                            let incoming = -up_direction * hit.distance;
                            position = new_position + incoming;

                            // remaining_motion -= step_forward; // FIXME: this may result in new motion being in the opposite direction.
                            remaining_motion -=
                                horizontal_dir * f32::min(step_forward_dist, horizontal_len);

                            // dbg!(remaining_motion);
                            did_step = true;
                            // break;
                        }

                        // Since we have stepped over the wall, the previous slope is now invalid.
                        // FIXME: maybe not?
                        // slope_index = SlopeIndex::First;
                    };
                }
            }
        }

        continue;

        // 1.5. Attempt step up slope.
        // if slope.is_floor() {
        // {
        // if let Ok(horizontal_dir) = Dir3::new(remaining_motion.reject_from(*up_direction)) {
        //     let step_up = up_direction * max_step_height;
        //     // let step_forward = horizontal_dir * (min_step_depth + skin_width);
        //     // let step_forward = horizontal_dir
        //     //     * (f32::min(min_step_depth, horizontal_dir.dot(remaining_motion)) + skin_width);
        //     let step_forward =
        //     //     horizontal_dir * (horizontal_dir.dot(remaining_motion) + skin_width);
        //     horizontal_dir * horizontal_dir.dot(remaining_motion);

        //     // Sweep down from the target step location.
        //     if let Some(hit) = spatial.cast_shape(
        //         shape,
        //         position + step_up + step_forward,
        //         rotation,
        //         -up_direction,
        //         &ShapeCastConfig {
        //             max_distance: max_step_height,
        //             target_distance: 0.0,
        //             compute_contact_on_penetration: true,
        //             ignore_origin_penetration: true,
        //         },
        //         filter,
        //     ) {
        //         {
        //             dbg!(hit.distance);
        //             // let incoming = -up_direction * (hit.distance - skin_width);
        //             let incoming = -up_direction * hit.distance;
        //             position += step_up + step_forward + incoming;
        //             remaining_motion -= step_forward; // FIXME: this may result in new motion being in the opposite direction.

        //             // Since we have stepped over the wall, the previous slope is now invalid.
        //             // FIXME: maybe not?
        //             slope_index = SlopeIndex::First;

        //             did_step = true;
        //         };
        //     }
        // }
        // }

        // 2. If not step, then slide.
        if !did_step {
            match slope_index {
                // When first colliding with the floor or a wall, we want to remove all motion
                // in the direction of the hit normal, i.e., project the motion on the normal plane.
                SlopeIndex::First => {
                    match slope {
                        // When moving on the floor, we want the speed to be the same regardless of angle.
                        // FIXME: this is probably wrong.
                        SlopePlane::Floor => {
                            // remaining_motion = remaining_motion.reject_from(hit.normal1);
                            remaining_motion
                                .reject_from(hit.normal1)
                                .try_normalize()
                                .map(|dir| {
                                    remaining_motion = dir * remaining_motion.length();
                                });
                            // velocity
                            //     .reject_from(hit.normal1)
                            //     .try_normalize()
                            //     .map(|dir| {
                            //         velocity = dir * velocity.length();
                            //     });
                        }
                        SlopePlane::Wall | SlopePlane::Roof => {
                            remaining_motion = remaining_motion.reject_from(hit.normal1);
                            // velocity = velocity.reject_from(hit.normal1);
                        }
                    }
                    slope_index = SlopeIndex::Second {
                        first_normal: hit.normal1,
                    };
                }
                // Since the previous iteration changed the motion direction, doing so again might
                // result in a new motion where no parts of it existed in the original desired motion.
                // This will result in the character "shaking" when walking into a corner.
                //
                // We solve this by projecting onto the "crease" normal of this and the previous hit instead.
                SlopeIndex::Second { first_normal }
                    if first_normal.dot(hit.normal1) < 1.0 - 1e-4 =>
                {
                    let crease = first_normal.cross(hit.normal1).normalize();
                    match slope {
                        SlopePlane::Floor => {
                            remaining_motion
                                .project_onto(crease)
                                .try_normalize()
                                .map(|dir| {
                                    remaining_motion = dir * remaining_motion.length();
                                });
                            velocity.project_onto(crease).try_normalize().map(|dir| {
                                // velocity = dir * velocity.length();
                            });
                        }
                        SlopePlane::Wall | SlopePlane::Roof => {
                            remaining_motion = remaining_motion.project_onto(crease);
                            // velocity = velocity.project_onto(crease);
                        }
                    }
                    slope_index = SlopeIndex::Third;
                }
                // When the character is sliding on three unique normals, all remaining motion is cancelled out.
                _ => {
                    remaining_motion = Vec3::ZERO;
                    velocity = Vec3::ZERO;
                }
            }
        }

        // 3. Snap to floor.
        // if floor_snap_length > 0.0 {
        //     if let Some(hit) = spatial.cast_shape(
        //         shape,
        //         position,
        //         rotation,
        //         -up_direction,
        //         &ShapeCastConfig {
        //             max_distance: dbg!(floor_snap_length) + skin_width,
        //             target_distance: skin_width,
        //             compute_contact_on_penetration: true,
        //             ignore_origin_penetration: true,
        //         },
        //         filter,
        //     ) {
        //         let incoming = -up_direction * (hit.distance - skin_width);
        //         position += incoming;
        //     }
        // }
    }

    // 3. Snap to floor.
    // if let Some(hit) = spatial.cast_shape(
    //     shape,
    //     position,
    //     rotation,
    //     -up_direction,
    //     &ShapeCastConfig {
    //         max_distance: dbg!(floor_snap_length) + skin_width,
    //         target_distance: skin_width,
    //         compute_contact_on_penetration: true,
    //         ignore_origin_penetration: true,
    //     },
    //     filter,
    // ) {
    //     let incoming = -up_direction * (hit.distance - skin_width);
    //     position += incoming;
    // }

    // 4. Solve collisions.
    // let mut overlaps = SmallVec::<[_; 8]>::new_const();
    // for _ in 0..12 {
    //     overlaps.clear();
    //     if let Some(motion) = solve_overlaps(
    //         &inflate_shape(shape, skin_width),
    //         position,
    //         rotation,
    //         up_direction,
    //         &mut overlaps,
    //         filter,
    //         spatial,
    //         |_| {},
    //     ) {
    //         position += motion;
    //     }
    //     for overlap in &overlaps {
    //         match SlopePlane::new(
    //             up_direction,
    //             Dir3::new_unchecked(overlap.normal),
    //             max_floor_angle,
    //         ) {
    //             SlopePlane::Floor => {
    //                 is_on_floor = true;
    //                 // velocity
    //                 //     .reject_from(overlap.normal)
    //                 //     .try_normalize()
    //                 //     .map(|dir| {
    //                 //         velocity = dir * velocity.length();
    //                 //     });
    //             }
    //             SlopePlane::Wall => {
    //                 is_on_wall = true;
    //                 velocity = velocity.reject_from(overlap.normal);
    //             }
    //             SlopePlane::Roof => {
    //                 is_on_roof = true;
    //                 velocity = velocity.reject_from(overlap.normal);
    //             }
    //         }
    //     }
    // }

    MovementOutput {
        position,
        velocity,
        remaining: remaining_motion,
        slide_distance: 0.0,
        travel_distance: 0.0,
    }
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
    mut on_hit: impl FnMut(Vec3),
    gizmos: &mut Gizmos,
) -> Vec3 {
    // TODO:
    // - [x] (ish) solve all overlaps in one go
    // - [x] Snap to floor
    // - [x] (ish) stair stepping
    // - [ ] slope handling
    // - [ ] floor velocity
    // - [ ] only snap to floor when we touched the floor previously and no longer do,
    // - [x] snap to floor should run after slide
    // - [ ] also return new velocity
    // - [ ] stepping configuration, min width, max height, exclude dynamic bodies..
    // - [ ] detect grounded at starting pos.

    let skin_width = config.skin_width.eval(shape);
    let max_step_height = config.max_step_height.eval(shape);
    let min_step_width = config.min_step_width.eval(shape);

    let inflated_shape = inflate_shape(shape, skin_width);

    let mut position = origin;
    let mut remaining_motion = target_motion;

    let mut is_on_floor = false;
    let mut is_on_wall = false;
    let mut is_on_roof = false;

    enum CollisionIndex {
        First,
        Second { first_normal: Vec3 },
        Third,
    }

    let mut collision_index = CollisionIndex::First;

    // Perform move and slide,
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
                target_distance: skin_width,
                compute_contact_on_penetration: true,
                ignore_origin_penetration: true,
            },
            filter,
        ) else {
            if config.apply_remaining_motion {
                position += mem::take(&mut remaining_motion);
            }
            break;
        };

        gizmos.arrow(hit.point1, hit.point1 + hit.normal1 * 4.0, RED);

        let incoming = direction * (hit.distance - skin_width); // Move a little bit away from surface
        // let incoming = direction * f32::max(0.0, hit.distance - skin_width);

        position += incoming;
        remaining_motion -= incoming;

        let slope = Slope::new(config.up, Dir3::new_unchecked(hit.normal1));

        gizmos.sphere(Isometry3d::from_translation(hit.point1), 2.0, BLACK);

        match slope.plane(config.max_slope_angle) {
            SlopePlane::Floor => is_on_floor = true,
            SlopePlane::Wall => is_on_wall = true,
            SlopePlane::Roof => is_on_roof = true,
        }

        // Attempt steup up.
        if true || !slope.is_floor(config.max_slope_angle) {
            let min_width = 1.0;
            let horizontal_motion = remaining_motion.reject_from(*config.up);
            // let horizontal_motion_len_sq = horizontal_motion.length_squared();
            // dbg!(horizontal_motion_len_sq);
            if horizontal_motion.length_squared() > 1e-6 {
                let horizontal_dir = horizontal_motion.normalize();
                let target_step_pos = position
                    + config.up * max_step_height
                    + horizontal_dir * (min_width + skin_width);
                if let Some(hit) = spatial.cast_shape(
                    // &inflated_shape,
                    shape,
                    target_step_pos,
                    rotation,
                    -config.up,
                    &ShapeCastConfig {
                        // max_distance: max_step_height,
                        max_distance: max_step_height + skin_width,
                        target_distance: 0.0,
                        compute_contact_on_penetration: true,
                        ignore_origin_penetration: true,
                    },
                    filter,
                ) {
                    let slope = Slope::new(config.up, Dir3::new_unchecked(hit.normal1));
                    if slope.is_floor(config.max_slope_angle) {
                        let incoming = -config.up * (hit.distance - skin_width);
                        position = target_step_pos + incoming;
                        // remaining_motion -= horizontal_motion;
                        remaining_motion -= horizontal_dir * (min_width + skin_width);
                        // remaining_motion = remaining_motion.reject_from(hit.normal1); // Not sure about this.

                        // on_hit(hit.normal1);

                        collision_index = CollisionIndex::First;
                        continue;
                    }
                }
            }
        }

        on_hit(hit.normal1);

        match collision_index {
            // One collision, project on plane.
            CollisionIndex::First => {
                remaining_motion = remaining_motion.reject_from(hit.normal1);

                collision_index = CollisionIndex::Second {
                    first_normal: hit.normal1,
                };
            }
            // Two collisions, project on crease normal.
            CollisionIndex::Second { first_normal } => {
                // Two collisions may have the same normal,
                // this is most likely a bug but let's ignore it for now.
                if first_normal.dot(hit.normal1) > 1.0 - 1e-4 {
                    remaining_motion = remaining_motion.reject_from(hit.normal1);
                    continue;
                }

                let crease = first_normal.cross(hit.normal1).normalize();

                remaining_motion = remaining_motion.project_onto(crease);

                collision_index = CollisionIndex::Third;

                gizmos.arrow(hit.point1, hit.point1 + crease * 4.0, LIGHT_BLUE);
            }
            // Three collisions, no motion is possible.
            CollisionIndex::Third => {
                remaining_motion = Vec3::ZERO; // This is not used but maybe it will be, idk.
                break;
            }
        }
    }

    // Attempt to solve all overlaps.
    let mut overlaps = SmallVec::<[_; 8]>::new_const();
    for _ in 0..16 {
        overlaps.clear();
        if let Some(motion) = solve_overlaps(
            &inflated_shape,
            position,
            rotation,
            config.up,
            &mut overlaps,
            filter,
            spatial,
            |normal| {
                let slope = Slope::new(config.up, Dir3::new_unchecked(normal));
                if slope.is_floor(config.max_slope_angle) {
                    is_on_floor = true;
                }
                on_hit(normal);
            },
        ) {
            position += motion;
        }
    }

    let floor_snap_distance = config.floor_snap_distance.eval(shape);

    // Snap to floor.
    if !is_on_floor && floor_snap_distance > 0.0 {
        if let Some(hit) = spatial.cast_shape(
            &inflated_shape,
            position,
            rotation,
            -config.up,
            &ShapeCastConfig {
                max_distance: floor_snap_distance,
                target_distance: 0.0,
                compute_contact_on_penetration: true,
                ignore_origin_penetration: true,
            },
            filter,
        ) {
            let incoming = -config.up * hit.distance;
            position += incoming;
        }
    }

    position
}

fn handle_stairs(
    shape: &Collider,
    position: &mut Vec3,
    remaining_motion: &mut Vec3,
    rotation: Quat,
    max_step_height: f32,
    min_step_width: f32,
    max_slope_angle: f32,
    skin_width: f32,
    up: Dir3,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    gizmos: &mut Gizmos,
) -> bool {
    let horizontal_motion = remaining_motion.reject_from(*up);
    let horizontal_len = horizontal_motion.length();
    let Some(horizontal_dir) = horizontal_motion.try_normalize() else {
        return false;
    };

    let start = *position + up * max_step_height + horizontal_dir * (min_step_width + skin_width);

    gizmos.sphere(Isometry3d::from_translation(start), 1.0, WHITE);

    if let Some(hit) = spatial.cast_shape(
        shape,
        start,
        rotation,
        -up,
        &ShapeCastConfig {
            max_distance: max_step_height + skin_width,
            target_distance: skin_width,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: true,
        },
        filter,
    ) {
        let new_position = start - up * (hit.distance - skin_width);

        gizmos.sphere(Isometry3d::from_translation(new_position), 1.0, BLACK);

        let normal = spatial
            .cast_ray(hit.point1 + up * 0.1, -up, 1.0, true, filter)
            .map(|ray_hit| ray_hit.normal)
            .unwrap_or(hit.normal1);

        let slope = Slope::new(up, Dir3::new_unchecked(normal));

        match slope.plane(max_slope_angle) {
            SlopePlane::Floor => {
                gizmos.arrow(hit.point1, hit.point1 + normal * 2.0, GREEN);
                gizmos.sphere(Isometry3d::from_translation(hit.point1), 1.0, GREEN);

                *position = new_position;
                *remaining_motion -= horizontal_dir * f32::min(horizontal_len, min_step_width);

                // *remaining_motion -= horizontal_dir * (min_step_width + skin_width);

                return true;
            }
            SlopePlane::Wall | SlopePlane::Roof => {
                gizmos.arrow(hit.point1, hit.point1 + normal * 2.0, RED);
                gizmos.sphere(Isometry3d::from_translation(hit.point1), 1.0, RED);
                return false;
            }
        }
    }

    false
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
    up: Dir3,
    overlaps: &mut SmallVec<T>,
    filter: &SpatialQueryFilter,
    spatial: &SpatialQuery,
    mut on_hit: impl FnMut(Vec3),
) -> Option<Vec3> {
    spatial.shape_hits_callback(
        &inflated_shape,
        position,
        rotation,
        -up,
        &ShapeCastConfig {
            max_distance: 0.0, // Use this for snaping to floor.
            target_distance: 0.0,
            compute_contact_on_penetration: true,
            ignore_origin_penetration: false,
        },
        filter,
        |hit| {
            let start = rotation * hit.point2 + position;
            let depth_sq = start.distance_squared(hit.point1);
            if depth_sq > 1e-6 {
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

    // Solve each overlap.
    for i in 0..overlaps.len() {
        let overlap = overlaps[i];

        if overlap.depth < 1e-4 {
            continue;
        }

        on_hit(overlap.normal);

        motion += overlap.depth * overlap.normal;

        // Make sure the next overlaps doesn't move the character further than neccessary.
        for next_hit in &mut overlaps[i + 1..] {
            if next_hit.depth < 1e-4 {
                continue;
            }
            let accounted_for = f32::max(0.0, overlap.normal.dot(next_hit.normal) * overlap.depth);
            next_hit.depth = f32::max(0.0, next_hit.depth - accounted_for);
        }
    }

    Some(motion)
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
    let skin_width = config.skin_width.eval(shape);

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
