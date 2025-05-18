use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::{color::palettes::css::*, input::InputSystem, prelude::*};
use debug::{CharacterGizmos, DebugHit, DebugMode, DebugMotion, DebugPoint};
use ground::{CharacterGrounding, GroundSurface, ground_check, is_walkable};
use projection::{CollisionState, Surface, align_with_surface, project_velocity};
use sweep::{MovementConfig, MovementImpact, collide_and_slide, step_check, sweep};

pub mod debug;
pub(crate) mod ground;
pub(crate) mod projection;
pub(crate) mod sweep;

pub struct KinematicCharacterPlugin;

impl Plugin for KinematicCharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((debug::plugin,));

        app.add_systems(PreUpdate, clear_movement_input.before(InputSystem));

        app.add_systems(
            FixedUpdate,
            (character_forces, update_character_filter, movement_input).chain(),
        );

        app.add_systems(
            FixedPostUpdate,
            (
                physics_interactions.before(PhysicsSet::Prepare),
                move_character.after(PhysicsSet::Sync),
            ),
        );

        app.add_systems(
            PhysicsSchedule,
            depenetrate_character.in_set(NarrowPhaseSet::Last),
        );
    }
}

pub(crate) fn update_character_filter(
    mut query: Query<(Entity, &mut CharacterFilter, &CollisionLayers)>,
    sensors: Query<Entity, With<Sensor>>,
) {
    for (entity, mut filter, collidion_layers) in &mut query {
        // Filter out any entities that's not in the character's collision filter
        filter.0.mask = collidion_layers.filters;

        // Filter out all sensor entities along with the character entity
        filter.0.excluded_entities.clear();
        filter
            .0
            .excluded_entities
            .extend(sensors.iter().chain([entity]));
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
        &mut Character,
        &CharacterMovement,
        Has<DebugMode>,
    )>,
    time: Res<Time>,
) {
    for (move_input, mut move_acceleration, mut character, movement, debug_mode) in &mut query {
        let Ok((direction, throttle)) = Dir3::new_and_length(move_input.value) else {
            continue;
        };

        if debug_mode {
            character.velocity = direction * movement.target_speed * throttle;
            continue;
        }

        let max_acceleration = match character.grounded() {
            true => movement.ground_acceleratin,
            false => movement.air_acceleratin,
        };

        let mut move_accel = acceleration(
            character.velocity,
            *direction,
            max_acceleration * throttle,
            movement.target_speed * throttle,
            time.delta_secs(),
        );

        if let Some(ground) = character.grounding.stable_surface() {
            move_accel = align_with_surface(move_accel, *ground.normal, *character.up);
        }

        character.velocity += move_accel;
    }
}

pub(crate) fn depenetrate_character(
    mut overlaps: Local<Vec<(Dir3, f32)>>,
    mut gizmos: Gizmos<CharacterGizmos>,
    collisions: Collisions,
    mut query: Query<(Entity, &mut Character, &GroundingSettings, &mut Transform)>,
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

        let (entity, mut character, grounding_settings, mut transform) =
            if let Ok(character) = query.get_mut(rb1) {
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

            let ground_normal = character.grounding.normal();

            let surface = Surface::new(hit_normal, grounding_settings.max_angle, character.up);
            let obstruction_normal = surface
                .obstruction_normal(ground_normal, character.up)
                .unwrap();

            for contact in &manifold.points {
                let depth = contact.penetration * hit_normal.dot(*obstruction_normal);

                match overlaps.binary_search_by(|(_, d)| depth.total_cmp(d)) {
                    Ok(index) => {
                        if overlaps[index].0.dot(*obstruction_normal) > 1.0 - 1e-4 {
                            overlaps.push((obstruction_normal, depth));
                            overlaps.swap_remove(index);
                        } else {
                            overlaps.insert(index, (obstruction_normal, depth));
                        }
                    }
                    Err(index) => {
                        overlaps.insert(index, (obstruction_normal, depth));
                    }
                }
            }

            if character.velocity.dot(hit_normal) > 0.0 {
                continue;
            }

            character.velocity = project_velocity(
                character.velocity,
                *obstruction_normal,
                surface.is_walkable,
                ground_normal,
                character.up,
            );

            if surface.is_walkable {
                character.grounding = GroundSurface::new(other, hit_normal).into();
            }
        }

        for i in 0..overlaps.len() {
            let (direction, depth) = overlaps[i];

            if depth <= 0.0 {
                continue;
            }

            gizmos.line_gradient(
                transform.translation,
                transform.translation - direction * depth,
                CRIMSON,
                CRIMSON.with_alpha(0.0),
            );

            transform.translation += direction * depth;

            for j in i..overlaps.len() {
                let (next_direction, ref mut next_depth) = overlaps[j];

                let fixed = f32::max(0.0, direction.dot(*next_direction) * depth);
                *next_depth -= fixed;
            }
        }
    }
}

fn physics_interactions(
    spatial_query: SpatialQuery,
    mut query: Query<
        (
            &mut Character,
            &mut Transform,
            &Collider,
            &ComputedMass,
            &CharacterFilter,
        ),
        Without<Sensor>,
    >,
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

/// Triggered when the character becomes grounded during a movement update.
///
/// This is only triggered for the last ground the character touched during the update and will not be triggered
/// if the character was already grounded prior to the start of the update.
#[derive(Event, Deref)]
pub struct OnGroundEnter(pub GroundSurface);

/// Triggered when the character becomes ungrounded during a movement update.
///
/// This is only triggered if the character is ungrounded at the end of the update.
#[derive(Event, Deref)]
pub struct OnGroundLeave(pub GroundSurface);

pub(crate) fn move_character(
    mut commands: Commands,
    spatial_query: SpatialQuery,
    mut query: Query<(
        Entity,
        &mut Character,
        &GroundingSettings,
        &SteppingSettings,
        &mut Transform,
        &Collider,
        &ComputedMass,
        &CharacterFilter,
        Has<Sensor>,
        Option<&mut DebugMotion>,
        Has<DebugMode>,
    )>,
    mut bodies: Query<(&mut LinearVelocity, &ComputedMass, &RigidBody)>,
    mut collision_started_events: EventWriter<CollisionStarted>,
    mut collision_ended_events: EventWriter<CollisionEnded>,
    time: Res<Time>,
) -> Result {
    for (
        entity,
        mut character,
        grounding_settings,
        stepping_settings,
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

        let current_ground_normal = character.grounding.normal();

        if debug_mode {
            if let Some(lines) = debug_motion.as_mut() {
                lines.push(
                    0.0,
                    DebugPoint {
                        translation: transform.translation,
                        velocity: character.velocity,
                        hit: current_ground_normal.map(|normal| DebugHit {
                            point: transform.translation
                                + character.feet_position(collider, transform.rotation),
                            normal: *normal,
                            is_walkable: true,
                        }),
                    },
                );
            }
        }

        let duration = match debug_mode {
            true => 1.0,
            false => time.delta_secs(),
        };

        assert_eq!(
            None, grounding_settings.layer_mask,
            "custom layer masks for walkable ground is not supported"
        );
        assert_eq!(
            None, stepping_settings.layer_mask,
            "custom layer masks for steppable surfaces are not supported"
        );

        // When already grounded, add a small epsilon to make sure we don't randomly
        // loose grip when the ground angle matches the max angle perfectly
        let walkable_angle = match character.grounded() {
            true => grounding_settings.max_angle + 0.01,
            false => grounding_settings.max_angle,
        };

        let mut movement = collide_and_slide(
            collider,
            transform.translation,
            transform.rotation,
            character.velocity,
            current_ground_normal,
            MovementConfig {
                walkable_angle,
                up_direction: character.up,
                max_slide_count: character.max_slide_count,
                skin_width: character.skin_width,
            },
            &filter.0,
            &spatial_query,
            duration,
            |velocity, surface| {
                surface.project_velocity(velocity, current_ground_normal, character.up)
            },
            |state,
             MovementImpact {
                 end,
                 direction,
                 remaining_motion,
                 surface,
                 hit,
                 ..
             }| {
                let try_step = match stepping_settings.behaviour {
                    SteppingBehaviour::Never => false,
                    SteppingBehaviour::Grounded => !surface.is_walkable && character.grounded(),
                    SteppingBehaviour::Always => !surface.is_walkable,
                };

                if try_step {
                    if let Ok((direction, motion)) = Dir3::new_and_length(
                        direction.reject_from(*character.up) * remaining_motion,
                    ) {
                        if let Some((offset, hit)) = step_check(
                            collider,
                            end,
                            transform.rotation,
                            direction,
                            motion,
                            character.up,
                            grounding_settings.max_angle,
                            1.0,
                            0.1,
                            stepping_settings.max_height,
                            character.skin_width,
                            &spatial_query,
                            &filter.0,
                        ) {
                            state.velocity =
                                align_with_surface(state.velocity, hit.normal1, *character.up);
                            state.ground = Some(GroundSurface::new(hit.entity, hit.normal1));
                            state.offset += offset;

                            return false;
                        }
                    }
                }

                // Trigger collision events
                collision_started_events.write(CollisionStarted(entity, hit.entity));

                // For now, assume the collision is ended instantly which is probably the case with move and slide anyways
                collision_ended_events.write(CollisionEnded(entity, hit.entity));

                if let Ok((mut other_vel, other_mass, other_body)) = bodies.get_mut(hit.entity) {
                    if other_body.is_dynamic() {
                        other_vel.0 += direction.project_onto(hit.normal1)
                            * remaining_motion
                            * other_mass.inverse()
                            * character_mass.value();
                    }
                }

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
                                    is_walkable: surface.is_walkable,
                                }),
                            },
                        );
                    }
                }

                true
            },
        );

        let mut new_translation = transform.translation + movement.offset;

        // This causes the character to not become grounded when landing on ramps and velocity is pointing up
        // if !character.grounded() && character.velocity.dot(*character.up) > 1e-4 {
        //     movement.ground = None;
        // }

        if character.grounded() || movement.ground.is_some() {
            if let Some((ground_distance, ground)) = ground_check(
                collider,
                new_translation,
                transform.rotation,
                character.up,
                grounding_settings.max_distance,
                character.skin_width,
                walkable_angle,
                &spatial_query,
                &filter.0,
            ) {
                movement.ground = Some(ground);

                let mut hit_roof = !grounding_settings.snapping_enabled;

                if grounding_settings.snapping_enabled && ground_distance < 0.0 {
                    hit_roof = sweep(
                        collider,
                        transform.translation,
                        transform.rotation,
                        character.up,
                        -ground_distance,
                        character.skin_width,
                        &spatial_query,
                        &filter.0,
                        true,
                    )
                    .is_some();
                }

                if !hit_roof {
                    new_translation -= character.up * ground_distance;
                }
            } else {
                movement.ground = None;
            }
        }

        match (character.grounding.inner_surface, movement.ground) {
            (Some(ground), None) => {
                commands.entity(entity).trigger(OnGroundLeave(ground));
            }
            (None, Some(ground)) => {
                commands.entity(entity).trigger(OnGroundEnter(ground));
            }
            _ => {}
        }

        let draw_line = debug_mode
            || debug_motion.as_ref().is_some_and(|lines| {
                lines.points.back().is_none_or(|(_, point)| {
                    point.translation.distance_squared(new_translation) > 0.5
                })
            });

        if draw_line {
            let lines = debug_motion.as_mut().unwrap();

            lines.push(
                movement.remaining_time,
                DebugPoint {
                    translation: new_translation,
                    velocity: movement.velocity,
                    hit: movement.ground.map(|ground| DebugHit {
                        point: new_translation
                            + character.feet_position(collider, transform.rotation),
                        normal: *ground.normal,
                        is_walkable: true,
                    }),
                },
            );
        }

        if !debug_mode {
            transform.translation = new_translation;
            character.velocity = movement.velocity;

            character.grounding = CharacterGrounding::new(movement.ground);
        }
    }

    Ok(())
}

pub(crate) struct PlatformVelocity {
    pub current: Vec3,
    pub previous: Vec3,
}

pub(crate) struct CharacterVelocity(pub Vec3);

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct GroundingSettings {
    /// Mask for walkable ground
    pub layer_mask: Option<LayerMask>,
    /// Max walkable angle
    pub max_angle: f32,
    /// Max distance from the ground
    pub max_distance: f32,
    pub snapping_enabled: bool,
}

impl Default for GroundingSettings {
    fn default() -> Self {
        Self {
            layer_mask: None,
            max_angle: PI / 4.0,
            max_distance: 0.2,
            snapping_enabled: true,
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SteppingBehaviour {
    Never,
    Grounded,
    Always,
}

#[derive(Component, Debug, PartialEq, Clone, Copy)]
pub struct SteppingSettings {
    pub layer_mask: Option<LayerMask>,
    pub max_height: f32,
    pub behaviour: SteppingBehaviour,
}

impl Default for SteppingSettings {
    fn default() -> Self {
        Self {
            layer_mask: None,
            max_height: 0.25,
            behaviour: SteppingBehaviour::Grounded,
        }
    }
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
    GroundingSettings,
    SteppingSettings,
)]
pub struct Character {
    pub velocity: Vec3,
    pub up: Dir3,
    pub skin_width: f32,
    pub max_slide_count: u8,
    pub grounding: CharacterGrounding,
    pub previous_ground: Option<GroundSurface>,
}

impl Default for Character {
    fn default() -> Self {
        Self {
            velocity: Vec3::ZERO,
            up: Dir3::Y,
            skin_width: 0.02,
            max_slide_count: 8,
            grounding: CharacterGrounding::default(),
            previous_ground: None,
        }
    }
}

impl Character {
    /// Launch the character, clearing the grounded state if launched away from the `ground` normal.
    pub fn launch(&mut self, impulse: Vec3) {
        if let Some(ground) = self.grounding.stable_surface() {
            // Clear grounded if launched away from the ground
            if ground.normal.dot(impulse) > 0.0 {
                self.grounding.detach_from_surface();
            }
        }

        self.velocity += impulse
    }

    /// Launch the character on the `up` axis, overriding the downward velocity.
    pub fn jump(&mut self, impulse: f32) {
        // Remove vertical velocity
        self.velocity = self.velocity.reject_from(*self.up);

        // Push the character away from ramps
        if let Some(ground) = self.grounding.detach_from_surface() {
            let impulse = ground.normal * self.velocity.dot(*ground.normal).min(0.0);

            let vertical = impulse.project_onto(*self.up);
            let mut horizontal = impulse - vertical;

            // Reorient horizontal impulse to original velocity direction
            if let Ok(direction) = Dir3::new(self.velocity) {
                horizontal = horizontal.project_onto(*direction);
            }

            self.velocity -= horizontal + vertical;
        }

        self.velocity += self.up * impulse;
    }

    /// Returns `true` if the character is standing on the ground.
    pub fn grounded(&self) -> bool {
        self.grounding.is_stable()
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
