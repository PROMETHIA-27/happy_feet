use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::{
    color::palettes::css::{CRIMSON, LIGHT_GREEN, WHITE},
    input::InputSystem,
    prelude::*,
};
use debug::{CharacterGizmos, DebugHit, DebugMode, DebugMotion, DebugPoint};
use ground::{Ground, ground_check, is_walkable};
use project::{SlidePlanes, project_motion_on_ground, project_motion_on_wall};
use sweep::{CollideImpact, collide_and_slide, step_check, sweep};

pub mod debug;
pub(crate) mod ground;
pub(crate) mod project;
pub(crate) mod sweep;

pub struct KinematicCharacterPlugin;

impl Plugin for KinematicCharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((debug::plugin,));

        app.add_systems(PreUpdate, clear_movement_input.before(InputSystem));

        app.add_systems(
            FixedUpdate,
            (
                character_forces,
                update_character_filter,
                movement_input,
                movement_acceleration,
            )
                .chain(),
        );

        app.add_systems(
            FixedPostUpdate,
            (
                physics_interactions.before(PhysicsSet::Prepare),
                character_movement.after(PhysicsSet::Sync),
            ),
        );

        app.add_systems(
            PhysicsSchedule,
            character_depenetrate.in_set(NarrowPhaseSet::Last),
        );

        // app.add_systems(
        //     PhysicsSchedule,
        //     (
        //         // character_depenetrate.in_set(NarrowPhaseSet::Update),
        //         character_movement.in_set(PhysicsStepSet::Last),
        //     ),
        // );
    }
}

pub(crate) fn update_character_filter(
    mut query: Query<(Entity, &mut CharacterFilter, &CollisionLayers)>,
    sensors: Query<Entity, With<Sensor>>,
    // bodies: Query<(Entity, &RigidBody)>,
) {
    for (entity, mut filter, collidion_layers) in &mut query {
        // Filter out any entities that's not in the character's collision filter
        filter.0.mask = collidion_layers.filters.into();

        // Filter out all sensor entities along with the character entity
        filter.0.excluded_entities.clear();
        filter
            .0
            .excluded_entities
            .extend(sensors.iter().chain([entity]));

        // filter.0.excluded_entities.extend(
        //     bodies
        //         .iter()
        //         .filter(|(_, r)| r.is_dynamic())
        //         .map(|(e, _)| e),
        // );
    }
}

pub(crate) fn character_forces(
    mut query: Query<(&mut Character, &CharacterMovement)>,
    time: Res<Time>,
) {
    for (mut character, movement) in &mut query {
        character.velocity *= drag_factor(movement.drag, time.delta_secs());

        match character.grounded() {
            true => {
                let f = friction_factor(character.velocity, movement.friction, time.delta_secs());
                character.velocity *= f;
            }
            false => {
                character.velocity += movement.gravity * time.delta_secs();
            }
        }
    }
}

pub(crate) fn clear_movement_input(mut query: Query<&mut MoveInput>) {
    for mut move_input in &mut query {
        move_input.update();
    }
}

pub(crate) fn movement_input(
    mut query: Query<(
        &MoveInput,
        &mut MoveAcceleration,
        &Character,
        &CharacterMovement,
        Has<DebugMode>,
    )>,
    time: Res<Time>,
) {
    for (move_input, mut move_acceleration, character, movement, debug_mode) in &mut query {
        let Ok((direction, throttle)) = Dir3::new_and_length(move_input.value) else {
            continue;
        };

        if debug_mode {
            move_acceleration.0 = direction * movement.target_speed * throttle;
            continue;
        }

        let max_acceleration = match character.grounded() {
            true => movement.ground_acceleratin,
            false => movement.air_acceleratin,
        };

        move_acceleration.0 += acceleration(
            character.velocity,
            *direction,
            max_acceleration * throttle,
            movement.target_speed * throttle,
            time.delta_secs(),
        );
    }
}

pub(crate) fn movement_acceleration(
    spatial_query: SpatialQuery,
    mut query: Query<(
        &mut MoveAcceleration,
        &mut Character,
        &mut Transform,
        &Collider,
        &CharacterFilter,
        &mut SlidePlanes,
        Has<Sensor>,
        Has<DebugMode>,
    )>,
    time: Res<Time>,
) {
    for (
        mut move_accel,
        mut character,
        mut transform,
        collider,
        filter,
        mut planes,
        is_sensor,
        debug_mode,
    ) in &mut query
    {
        let mut move_accel = move_accel.take();

        if let Some(ground) = character.ground {
            // move_accel = move_accel.reject_from_normalized(*ground.normal);
            move_accel = project_motion_on_ground(move_accel, ground.normal, character.up);
        }

        if debug_mode {
            character.velocity = move_accel;
            continue;
        }

        if is_sensor {
            character.velocity += move_accel;
            continue;
        }

        character.velocity += move_accel;
        continue;

        // Sweep in the movement direction to find a plane to project acceleration on
        // This is a seperate step because trying to do this in the `move_and_slide` callback
        // results in "sticking" to the wall rather than sliding down at the expected rate
        let Ok(original_direction) = Dir3::new(move_accel) else {
            continue;
        };

        let out = collide_and_slide(
            collider,
            transform.translation,
            transform.rotation,
            move_accel,
            8,
            character.skin_width,
            &filter.0,
            &spatial_query,
            time.delta_secs(),
            |state, CollideImpact { hit, .. }| {
                if let Some(ground) = Ground::new_if_walkable(
                    hit.entity,
                    hit.normal1,
                    character.up,
                    character.walkable_angle,
                ) {
                    character.ground = Some(ground);
                }

                if planes.push(hit.normal1) {
                    if planes.is_corner() {
                        state.velocity = Vec3::ZERO;
                        return false;
                    }

                    for normal in planes.normals() {
                        if is_walkable(hit.normal1, *character.up, character.walkable_angle) {
                            state.velocity =
                                project_motion_on_ground(state.velocity, normal, character.up);
                        } else {
                            state.velocity =
                                project_motion_on_wall(state.velocity, normal, character.up);
                        }
                    }

                    for crease in planes.creases() {
                        state.velocity = state.velocity.project_onto_normalized(*crease);
                    }
                }

                false
            },
        );

        transform.translation += out.offset;
        move_accel = out.velocity;

        character.velocity += move_accel;
    }
}

// FIXME: doesn't work
pub(crate) fn character_depenetrate(
    mut overlaps: Local<Vec<(Dir3, f32)>>,
    mut gizmos: Gizmos<CharacterGizmos>,
    collisions: Collisions,
    mut query: Query<(Entity, &mut Character, &mut Transform)>,
    colliders: Query<&ColliderOf, Without<Sensor>>,
) {
    for contacts in collisions.iter() {
        overlaps.clear();

        // Get the rigid body entities of the colliders (colliders could be children)
        let Ok([&ColliderOf { body: rb1 }, &ColliderOf { body: rb2 }]) =
            colliders.get_many([contacts.collider1, contacts.collider2])
        else {
            continue;
        };

        let other: Entity;

        let (entity, mut character, mut transform) = if let Ok(character) = query.get_mut(rb1) {
            other = rb2;
            character
        } else if let Ok(character) = query.get_mut(rb2) {
            other = rb1;
            character
        } else {
            continue;
        };

        // TODO: crease / corner handling

        for manifold in &contacts.manifolds {
            let hit_normal = match entity == rb1 {
                true => -manifold.normal,
                false => manifold.normal,
            };

            let stable_on_hit = is_walkable(hit_normal, *character.up, character.walkable_angle);

            let stable_ground = character.ground.map(|g| *g.normal);

            let obstruction_normal =
                obstruction_normal(stable_ground, hit_normal, stable_on_hit, *character.up);

            // let obstruction_normal = hit_normal;

            for contact in &manifold.points {
                let Ok((direction, magnitude)) =
                    Dir3::new_and_length(hit_normal.project_onto(obstruction_normal))
                else {
                    continue;
                };

                // let depth = contact.penetration * magnitude + character.skin_width;
                let depth = contact.penetration * magnitude;

                match overlaps.binary_search_by(|(_, d)| depth.total_cmp(&d)) {
                    Ok(index) => {
                        if overlaps[index].0.dot(*direction) > 1.0 - 1e-4 {
                            overlaps.push((direction, depth));
                            overlaps.swap_remove(index);
                        } else {
                            overlaps.insert(index, (direction, depth));
                        }
                    }
                    Err(index) => {
                        overlaps.insert(index, (direction, depth));
                    }
                }
            }

            if character.velocity.dot(hit_normal) > 0.0 {
                continue;
            }

            character.velocity = project_velocity(
                character.velocity,
                stable_ground,
                *character.up,
                obstruction_normal,
                stable_on_hit,
            );

            if stable_on_hit {
                character.ground = Some(Ground {
                    entity: other,
                    normal: Dir3::new(hit_normal).unwrap(),
                });
            }
        }

        let mut fixup = Vec3::ZERO;

        for i in 0..overlaps.len() {
            let (direction, depth) = overlaps[i];

            let color = match depth > 0.0 {
                true => CRIMSON,
                false => LIGHT_GREEN,
            };

            gizmos.line_gradient(
                transform.translation,
                transform.translation - direction * depth,
                color,
                color.with_alpha(0.0),
            );

            if depth <= 0.0 {
                continue;
            }

            fixup += direction * depth;

            for j in i..overlaps.len() {
                let (next_direction, ref mut next_depth) = overlaps[j];

                let fixed = f32::max(0.0, direction.dot(*next_direction) * depth);
                *next_depth -= fixed;
            }
        }

        let Ok((direction, depth)) = Dir3::new_and_length(fixup) else {
            continue;
        };

        gizmos.line_gradient(
            transform.translation,
            transform.translation + direction * depth,
            WHITE,
            WHITE.with_alpha(0.0),
        );

        transform.translation += direction * depth;
    }
}

fn physics_interactions(
    spatial_query: SpatialQuery,
    mut query: Query<(
        &mut Character,
        &mut Transform,
        &Collider,
        &ComputedMass,
        &CharacterFilter,
    )>,
    mut bodies: Query<(&mut LinearVelocity, &ComputedMass, &RigidBody)>,
    time: Res<Time>,
) {
    for (mut character, mut transform, collider, character_mass, filter) in &mut query {
        let Ok((direction, max_distance)) =
            Dir3::new_and_length(character.velocity * time.delta_secs())
        else {
            continue;
        };

        let Some((distance, hit)) = sweep(
            collider,
            transform.translation,
            transform.rotation,
            direction,
            max_distance,
            character.skin_width,
            &spatial_query,
            &filter.0,
            false,
        ) else {
            continue;
        };

        let Ok((mut other_vel, other_mass, other_body)) = bodies.get_mut(hit.entity) else {
            continue;
        };

        // transform.translation += direction * distance;

        if other_body.is_dynamic() {
            let remaining = max_distance - distance;

            other_vel.0 += direction.project_onto(hit.normal1)
                * remaining
                * other_mass.inverse()
                * character_mass.value();

            character.velocity -= direction.project_onto(hit.normal1) * distance;
        }
    }
}

pub(crate) fn character_movement(
    mut gizmos: Gizmos<CharacterGizmos>,
    spatial_query: SpatialQuery,
    mut query: Query<(
        &mut Character,
        &mut Transform,
        &Collider,
        &ComputedMass,
        &CharacterFilter,
        Has<Sensor>,
        Option<&mut DebugMotion>,
        Has<DebugMode>,
    )>,
    mut bodies: Query<(&mut LinearVelocity, &ComputedMass, &RigidBody)>,
    time: Res<Time>,
) -> Result {
    for (
        mut character,
        mut transform,
        collider,
        character_mass,
        filter,
        is_sensor,
        mut debug_motion,
        debug_mode,
    ) in &mut query
    {
        if is_sensor {
            transform.translation += character.velocity * time.delta_secs();
            continue;
        }

        if debug_mode {
            if let Some(lines) = debug_motion.as_mut() {
                lines.push(
                    0.0,
                    DebugPoint {
                        translation: transform.translation,
                        velocity: character.velocity,
                        hit: character.ground.map(|ground| DebugHit {
                            point: transform.translation
                                + character.feet_position(collider, transform.rotation),
                            normal: *ground.normal,
                        }),
                    },
                );
            }
        }

        let mut new_ground = None;

        let duration = match debug_mode {
            true => 1.0,
            false => time.delta_secs(),
        };

        let mut plane_index = 0;
        let mut previous_hit_normal = None;
        let mut previous_velocity = character.velocity;

        let out = collide_and_slide(
            collider,
            transform.translation,
            transform.rotation,
            character.velocity,
            8,
            character.skin_width,
            &filter.0,
            &spatial_query,
            duration,
            |state,
             CollideImpact {
                 hit,
                 end,
                 direction,
                 incoming_motion,
                 remaining_motion,
                 ..
             }| {
                let velocity_before_projection = state.velocity;
                let current_ground = character.ground.map(|g| *g.normal);

                let hit_is_walkable =
                    is_walkable(hit.normal1, *character.up, character.walkable_angle);
                let obstruction_normal =
                    obstruction_normal(current_ground, hit.normal1, hit_is_walkable, *character.up);

                if !hit_is_walkable && character.grounded() {
                    if let Some((offset, hit)) = step_check(
                        &mut gizmos,
                        collider,
                        end,
                        transform.rotation,
                        direction,
                        remaining_motion,
                        character.up,
                        character.walkable_angle,
                        1.0,
                        0.1,
                        character.step_height,
                        character.skin_width,
                        &spatial_query,
                        &filter.0,
                    ) {
                        new_ground = Some(Ground {
                            entity: hit.entity,
                            normal: hit.normal1.try_into().unwrap(),
                        });

                        state.offset += offset;

                        info!("DID STEP!!!");

                        return false;
                    }
                }

                if let Ok((mut other_vel, other_mass, other_body)) = bodies.get_mut(hit.entity) {
                    if other_body.is_dynamic() {
                        other_vel.0 += direction.project_onto(hit.normal1)
                            * remaining_motion
                            * other_mass.inverse()
                            * character_mass.value();
                    }
                }

                if let Some(ground) = Ground::new_if_walkable(
                    hit.entity,
                    hit.normal1,
                    character.up,
                    character.walkable_angle,
                ) {
                    new_ground = Some(ground);

                    state.velocity = project_velocity(
                        state.velocity,
                        current_ground,
                        *character.up,
                        obstruction_normal,
                        hit_is_walkable,
                    );
                } else {
                    match plane_index {
                        // initial
                        0 => {
                            plane_index = 1;

                            state.velocity = project_velocity(
                                state.velocity,
                                current_ground,
                                *character.up,
                                obstruction_normal,
                                hit_is_walkable,
                            );
                        }
                        // found blocking plane
                        1 => {
                            let previous_hit_normal = previous_hit_normal.unwrap();
                            let previous_hit_is_stable = is_walkable(
                                previous_hit_normal,
                                *character.up,
                                character.walkable_angle,
                            );

                            if let Some(crease) = evaluate_crease(
                                state.velocity,
                                previous_velocity,
                                hit.normal1,
                                previous_hit_normal,
                                hit_is_walkable,
                                previous_hit_is_stable,
                                character.grounded(),
                            ) {
                                if character.grounded() {
                                    plane_index = 3;
                                    state.velocity = Vec3::ZERO;
                                } else {
                                    plane_index = 2;
                                    state.velocity = state.velocity.project_onto(crease);
                                }
                            } else {
                                state.velocity = project_velocity(
                                    state.velocity,
                                    current_ground,
                                    *character.up,
                                    obstruction_normal,
                                    hit_is_walkable,
                                );
                            }
                        }
                        // found blocking crease
                        2 => {
                            plane_index = 3;
                            state.velocity = Vec3::ZERO;
                        }
                        // found blocking corner
                        _ => {}
                    }
                }

                previous_hit_normal = Some(hit.normal1);
                previous_velocity = velocity_before_projection;

                if debug_mode {
                    if let Some(lines) = debug_motion.as_mut() {
                        let duration = duration - state.remaining_time;
                        lines.push(
                            duration,
                            DebugPoint {
                                translation: transform.translation + state.offset,
                                velocity: state.velocity,
                                hit: Some(DebugHit {
                                    point: hit.point1,
                                    normal: hit.normal1,
                                }),
                            },
                        );
                    }
                }

                false
            },
        );

        let mut new_translation = transform.translation + out.offset;
        let new_velocity = out.velocity;

        if !character.grounded() && character.velocity.dot(*character.up) > 1e-4 {
            new_ground = None;
        }

        if character.grounded() || new_ground.is_some() {
            if let Some((ground_distance, ground)) = ground_check(
                collider,
                new_translation,
                transform.rotation,
                character.up,
                character.ground_check_distance,
                character.skin_width,
                character.walkable_angle + 0.01,
                &spatial_query,
                &filter.0,
            ) {
                if character.grounded() {
                    new_ground = Some(ground);

                    let mut hit_roof = false;

                    if ground_distance < 0.0 {
                        if let Some(..) = sweep(
                            collider,
                            transform.translation,
                            transform.rotation,
                            character.up,
                            -ground_distance,
                            character.skin_width,
                            &spatial_query,
                            &filter.0,
                            true,
                        ) {
                            hit_roof = true;
                        }
                    }

                    if !hit_roof {
                        new_translation -= character.up * ground_distance;
                    }
                }
            } else {
                new_ground = None;
            }
        }

        let draw_line = debug_mode
            || debug_motion.as_ref().map_or(false, |lines| {
                lines.points.back().map_or(true, |(_, point)| {
                    point.translation.distance_squared(new_translation) > 0.5
                })
            });

        if draw_line {
            let lines = debug_motion.as_mut().unwrap();

            lines.push(
                out.remaining_time,
                DebugPoint {
                    translation: new_translation,
                    velocity: new_velocity,
                    hit: new_ground.map(|ground| DebugHit {
                        point: new_translation
                            + character.feet_position(collider, transform.rotation),
                        normal: *ground.normal,
                    }),
                },
            );
        }

        if !debug_mode {
            transform.translation = new_translation;
            character.velocity = new_velocity;
            character.ground = new_ground;
        }
    }

    Ok(())
}

fn evaluate_crease(
    current_velocity: Vec3,
    previous_velocity: Vec3,
    current_hit_normal: Vec3,
    previous_hit_normal: Vec3,
    current_hit_is_stable: bool,
    previous_hit_is_stable: bool,
    ground_is_stable: bool,
) -> Option<Vec3> {
    if ground_is_stable && current_hit_is_stable && previous_hit_is_stable {
        return None;
    }

    // Avoid calculations if the two planes are the same
    if current_hit_normal.dot(previous_hit_normal) > 1.0 - 1e-3 {
        return None;
    }

    let mut crease_direction = current_hit_normal
        .cross(previous_hit_normal)
        .normalize_or_zero();

    let current_normal_on_crease_plane = current_hit_normal
        .reject_from(crease_direction)
        .normalize_or_zero();
    let previous_normal_on_crease_plane = previous_hit_normal
        .reject_from(crease_direction)
        .normalize_or_zero();

    let entering_velocity_on_crease_plane = previous_velocity.reject_from(crease_direction);

    let dot_planes_on_crease_planes = current_normal_on_crease_plane.dot(previous_hit_normal);

    if dot_planes_on_crease_planes
        <= (entering_velocity_on_crease_plane.dot(-current_normal_on_crease_plane) + 1e-3)
        && dot_planes_on_crease_planes
            <= (entering_velocity_on_crease_plane.dot(-previous_normal_on_crease_plane) + 1e-3)
    {
        // Flip crease direction to make it representative of the real direction our velocity would be projected to
        if crease_direction.dot(current_velocity) < 0.0 {
            crease_direction = -crease_direction;
        }

        return Some(crease_direction);
    }

    None
}

fn obstruction_normal(
    current_ground: Option<Vec3>,
    hit_normal: Vec3,
    hit_is_walkable: bool,
    up: Vec3,
) -> Vec3 {
    if !hit_is_walkable {
        if let Some(ground_normal) = current_ground {
            let ground_left = ground_normal.cross(hit_normal).normalize_or_zero();
            return ground_left.cross(up).normalize_or_zero();
        }
    }

    hit_normal
}

fn project_velocity(
    mut velocity: Vec3,
    current_ground: Option<Vec3>,
    up: Vec3,
    obstruction_normal: Vec3,
    stable_on_hit: bool,
) -> Vec3 {
    if let Some(ground_normal) = current_ground {
        if stable_on_hit {
            // On stable slopes, simply reorient the movement without any loss
            velocity = aligned_with_surface(velocity, obstruction_normal, up);
        } else {
            // On blocking hits, project the movement on the obstruction while following the grounding plane
            let ground_right = obstruction_normal.cross(ground_normal).normalize_or_zero();
            let ground_up = ground_right.cross(obstruction_normal).normalize_or_zero();
            velocity = aligned_with_surface(velocity, ground_up, up);
            velocity = velocity.reject_from(obstruction_normal)
        }
    } else {
        if stable_on_hit {
            // Handle stable landing
            velocity = velocity.reject_from(up);
            velocity = aligned_with_surface(velocity, obstruction_normal, up);
        } else {
            // Handle generic obstruction
            velocity = velocity.reject_from(obstruction_normal);
        }
    }

    velocity
}

pub(crate) enum SweepState {
    None,
    Plane,
    Crease { previous_normal: Dir3 },
    Corner,
}

fn aligned_with_surface(vector: Vec3, normal: Vec3, up: Vec3) -> Vec3 {
    direction_aligned_with_surface(vector, normal, up) * vector.length()
}

fn direction_aligned_with_surface(vector: Vec3, normal: Vec3, up: Vec3) -> Vec3 {
    let right = vector.cross(up);
    normal.cross(right).normalize_or_zero()
}

pub(crate) struct CharacterVelocity {
    pub movement: Vec3,
    pub ground: Vec3,
}

pub(crate) struct GroundSettings {
    /// mask for walkable ground
    layers: Option<LayerMask>,
    /// max walkable angle
    max_angle: f32,
    /// max distance from the ground
    max_distance: f32,
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
#[require(
    RigidBody = RigidBody::Kinematic,
    Collider = Capsule3d::new(0.4, 1.0),
    CharacterFilter,
    CharacterMovement,
    MoveInput,
    MoveAcceleration,
    SlidePlanes,
    // SteppingBehaviour,
    DebugMotion,
)]
pub struct Character {
    pub velocity: Vec3,
    pub up: Dir3,
    pub skin_width: f32,
    pub max_slide_count: u8,
    pub ground: Option<Ground>,
    pub previous_ground: Option<Ground>,
    pub walkable_angle: f32,
    pub ground_check_distance: f32,
    pub step_height: f32,
    pub snap_to_ground: bool,
}

impl Default for Character {
    fn default() -> Self {
        Self {
            velocity: Vec3::ZERO,
            up: Dir3::Y,
            skin_width: 0.02,
            max_slide_count: 8,
            ground: None,
            previous_ground: None,
            walkable_angle: PI / 4.0,
            ground_check_distance: 0.1,
            step_height: 0.3,
            snap_to_ground: true,
        }
    }
}

impl Character {
    /// Launch the character, clearing the grounded state if launched away from the `ground` normal.
    pub fn launch(&mut self, impulse: Vec3) {
        if let Some(ground) = self.ground {
            // Clear grounded if launched away from the ground
            if ground.normal.dot(impulse) > 0.0 {
                self.ground = None;
            }
        }

        self.velocity += impulse
    }

    /// Launch the character on the `up` axis, overriding the downward velocity.
    pub fn jump(&mut self, impulse: f32) {
        // Remove vertical velocity
        self.velocity = self.velocity.reject_from(*self.up);

        // Push the character away from ramps
        if let Some(ground) = self.ground.take() {
            self.velocity -= ground.normal * self.velocity.dot(*ground.normal).min(0.0);
        }

        self.velocity += self.up * impulse;
    }

    /// Set the current ground and update the prevoius ground with the old value
    pub fn set_ground(&mut self, ground: Option<Ground>) {
        self.previous_ground = self.ground;
        self.ground = ground;
    }

    /// Returns `true` if the character is standing on the ground.
    pub fn grounded(&self) -> bool {
        self.ground.is_some()
    }

    pub fn feet_position(&self, shape: &Collider, rotation: Quat) -> Vec3 {
        let aabb = shape.aabb(Vec3::ZERO, rotation);
        let down = aabb.min.dot(*self.up) - self.skin_width;
        self.up * down
    }
}

/// The next movement acceleration of the character
#[derive(Component, Reflect, Default, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct MoveAcceleration(pub Vec3);

impl MoveAcceleration {
    pub fn take(&mut self) -> Vec3 {
        std::mem::take(&mut self.0)
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct CharacterMovement {
    pub target_speed: f32,
    pub ground_acceleratin: f32,
    pub air_acceleratin: f32,
    pub gravity: Vec3,
    pub friction: f32,
    pub drag: f32,
    pub jump_impulse: f32,
}

impl Default for CharacterMovement {
    fn default() -> Self {
        Self {
            target_speed: 8.0,
            ground_acceleratin: 100.0,
            friction: 60.0,
            drag: 0.01,
            jump_impulse: 7.0,
            air_acceleratin: 20.0,
            gravity: Vec3::Y * -20.0,
        }
    }
}

/// Cache the [`SpatialQueryFilter`] of the character to avoid re-allocating the excluded entities map every time it's used.
///
/// This has to be a seperate component because otherwise the `character` cannot be mutated during a `move_and_slide` loop.
#[derive(Component, Reflect, Default, Debug)]
#[reflect(Component)]
pub struct CharacterFilter(pub(crate) SpatialQueryFilter);

#[derive(Component, Reflect, Default, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct MoveInput {
    pub value: Vec3,
    previous: Vec3,
}

impl MoveInput {
    pub fn update(&mut self) -> Vec3 {
        self.previous = std::mem::take(&mut self.value);
        self.previous
    }

    pub fn set(&mut self, value: Vec3) {
        self.value = value;
    }

    pub fn previous(&self) -> Vec3 {
        self.previous
    }
}

#[must_use]
fn acceleration(
    velocity: Vec3,
    direction: Vec3,
    max_acceleration: f32,
    target_speed: f32,
    delta: f32,
) -> Vec3 {
    // Current speed in the desired direction.
    let current_speed = velocity.dot(direction);

    // No acceleration is needed if current speed exceeds target.
    if current_speed >= target_speed {
        return Vec3::ZERO;
    }

    // Clamp to avoid acceleration past the target speed.
    let accel_speed = f32::min(target_speed - current_speed, max_acceleration * delta);

    direction * accel_speed
}

#[must_use]
pub(crate) fn drag_factor(drag: f32, delta: f32) -> f32 {
    f32::exp(-drag * delta)
}

/// Constant acceleration in the opposite direction of velocity.
#[must_use]
pub(crate) fn friction_factor(velocity: Vec3, friction: f32, delta: f32) -> f32 {
    let speed_sq = velocity.length_squared();

    if speed_sq < 1e-4 {
        return 0.0;
    }

    f32::exp(-friction / speed_sq.sqrt() * delta)
}
