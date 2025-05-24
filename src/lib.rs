use std::mem;

use avian3d::prelude::*;
use bevy::{color::palettes::css::*, input::InputSystem, prelude::*};
use debug::{CharacterGizmos, DebugHit, DebugMode, DebugMotion, DebugPoint};
use ground::{Ground, Grounding, GroundingSettings, ground_check, is_walkable};
use projection::{CollisionState, Surface, align_with_surface, project_velocity};
use sweep::{CollideAndSlideConfig, MovementImpact, collide_and_slide, step_check, sweep};

pub mod debug;
pub mod ground;
pub(crate) mod projection;
pub(crate) mod sweep;

pub struct KinematicCharacterPlugin;

impl Plugin for KinematicCharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((debug::plugin,));

        app.add_systems(PreUpdate, clear_movement_input.before(InputSystem));

        app.add_systems(
            FixedUpdate,
            (
                (character_drag, character_friction, character_gravity),
                update_character_filter,
                character_acceleration,
            )
                .chain(),
        );

        app.add_systems(
            FixedPostUpdate,
            (
                (
                    make_character_dynamic, //
                    physics_interactions,
                )
                    .before(PhysicsSet::Prepare),
                (
                    make_character_kinematic,
                    move_character_with_platform,
                    move_character,
                )
                    .after(PhysicsSet::Sync)
                    .chain(),
            ),
        );

        app.add_systems(
            PhysicsSchedule,
            depenetrate_character.in_set(NarrowPhaseSet::Last),
        );

        app.add_observer(inherit_platform_velocity);
    }
}

pub(crate) fn clear_movement_input(mut query: Query<&mut MoveInput>) {
    for mut move_input in &mut query {
        move_input.update();
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

pub(crate) fn character_gravity(
    gravity: Res<Gravity>,
    mut query: Query<(
        &mut Character,
        Option<&Grounding>,
        Option<&CharacterGravity>,
        Option<&GravityScale>,
    )>,
    time: Res<Time>,
) {
    for (mut character, grounding, character_gravity, gravity_scale) in &mut query {
        if grounding.map_or(false, |g| g.is_grounded()) {
            continue;
        }

        let gravity = character_gravity.map(|g| g.0).unwrap_or(gravity.0)
            * gravity_scale.map(|g| g.0).unwrap_or(1.0);

        character.velocity += gravity * time.delta_secs();
    }
}

pub(crate) fn character_friction(
    default_friction: Res<DefaultFriction>,
    mut characters: Query<(&mut Character, &Grounding, &CharacterFriction)>,
    frictions: Query<&Friction>,
    colliders: Query<&ColliderOf>,
    time: Res<Time>,
) {
    for (mut character, grounding, character_friction) in &mut characters {
        let Some(ground) = grounding.ground() else {
            continue;
        };

        let mut friction_scale = default_friction.dynamic_coefficient;

        if let Ok(friction) = frictions.get(ground.entity) {
            friction_scale = friction.dynamic_coefficient;
        } else if let Ok(collider_of) = colliders.get(ground.entity) {
            if let Ok(friction) = frictions.get(collider_of.body) {
                friction_scale = friction.dynamic_coefficient;
            }
        }

        let f = friction_factor(
            character.velocity,
            character_friction.0 * friction_scale,
            time.delta_secs(),
        );

        character.velocity *= f;
    }
}

pub(crate) fn character_drag(mut query: Query<(&mut Character, &CharacterDrag)>, time: Res<Time>) {
    for (mut character, drag) in &mut query {
        character.velocity *= drag_factor(drag.0, time.delta_secs());
    }
}

pub(crate) fn character_acceleration(
    mut query: Query<(
        &MoveInput,
        &mut Character,
        Option<&Grounding>,
        &CharacterMovement,
        Has<DebugMode>,
    )>,
    time: Res<Time>,
) {
    for (move_input, mut character, grounding, movement, debug_mode) in &mut query {
        let Ok((direction, throttle)) = Dir3::new_and_length(move_input.value) else {
            continue;
        };

        if debug_mode {
            character.velocity = direction * movement.target_speed * throttle;
            continue;
        }

        let (direction, velocity) = match grounding.and_then(|g| g.normal()) {
            Some(normal) => (
                align_with_surface(*direction, *normal, *character.up),
                align_with_surface(character.velocity, *normal, *character.up),
            ),
            None => (*direction, character.velocity),
        };

        let move_accel = acceleration(
            velocity,
            direction,
            movement.acceleration * throttle,
            movement.target_speed * throttle,
            time.delta_secs(),
        );

        character.velocity += move_accel;
    }
}

pub(crate) fn depenetrate_character(
    mut overlaps: Local<Vec<(Dir3, f32)>>,
    mut gizmos: Gizmos<CharacterGizmos>,
    collisions: Collisions,
    mut query: Query<(
        Entity,
        &mut Character,
        Option<(&mut Grounding, &GroundingSettings)>,
        &mut Transform,
    )>,
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

        let (entity, mut character, mut grounding, mut transform) =
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

            let (surface, obstruction_normal, ground_normal) = match grounding.as_ref() {
                Some((grounding, grounding_settings)) => {
                    let surface =
                        Surface::new(hit_normal, grounding_settings.max_angle, character.up);
                    let ground_normal = grounding.normal();
                    let obstruction_normal = surface
                        .obstruction_normal(ground_normal, character.up)
                        .unwrap();
                    (surface, obstruction_normal, ground_normal)
                }
                None => {
                    let surface = Surface {
                        normal: Dir3::new(hit_normal).unwrap(),
                        is_walkable: false,
                    };
                    (surface, surface.normal, None)
                }
            };

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
                if let Some((grounding, _)) = grounding.as_mut() {
                    **grounding = Ground::new(other, hit_normal).into();
                }
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

fn make_character_dynamic(
    mut query: Query<(&mut RigidBody, &mut LockedAxes, &mut GravityScale), With<Character>>,
) {
    for (mut rigidbody, mut locked_axes, mut gravity_scale) in &mut query {
        *rigidbody = RigidBody::Dynamic;
        gravity_scale.0 = 0.0;
        *locked_axes = LockedAxes::ROTATION_LOCKED;
    }
}

fn make_character_kinematic(
    mut query: Query<(&mut RigidBody, &mut LinearVelocity), With<Character>>,
) {
    for (mut rigidbody, mut linear_velocity) in &mut query {
        *rigidbody = RigidBody::Kinematic;
        linear_velocity.0 = Vec3::ZERO;
    }
}

fn physics_interactions(
    spatial_query: SpatialQuery,
    mut commands: Commands,
    characters: Query<
        (
            Entity,
            &Character,
            Option<&Grounding>,
            &Transform,
            &Collider,
            &ComputedMass,
            &CharacterFilter,
        ),
        Without<Sensor>,
    >,
    mut bodies: Query<(
        &mut LinearVelocity,
        &mut AngularVelocity,
        &ComputedMass,
        &ComputedAngularInertia,
        &GlobalTransform,
        &ComputedCenterOfMass,
        &RigidBody,
        &Collider,
    )>,
    time: Res<Time>,
) {
    for (
        character_entity,
        character,
        grounding,
        character_transform,
        character_collider,
        character_mass,
        filter,
    ) in &characters
    {
        let Ok((sweep_direction, target_sweep_distance)) =
            Dir3::new_and_length(character.velocity * time.delta_secs())
        else {
            continue;
        };

        let Some(hit) = sweep(
            character_collider,
            character_transform.translation,
            character_transform.rotation,
            sweep_direction,
            target_sweep_distance,
            character.skin_width,
            &spatial_query,
            &filter.0,
            false,
        ) else {
            continue;
        };

        let Ok((
            mut linear_velocity,
            mut angular_velocity,
            mass,
            inertia,
            transform,
            center_of_mass,
            rb,
            _,
        )) = bodies.get_mut(hit.entity)
        else {
            continue;
        };

        if !rb.is_dynamic() {
            continue;
        }

        // Don't push the body the character is standing on
        if let Some(grounding) = grounding {
            if grounding.entity() == Some(hit.entity) {
                continue;
            }
        }

        // We have to limit the impulse applied to bodies so that small bodies don't glitch out of existence

        let depth = 1.0 - hit.distance / target_sweep_distance;

        let Ok(impulse_direction) = Dir3::new(-hit.normal) else {
            continue;
        };

        let incoming_speed = character.velocity.dot(*impulse_direction);

        if incoming_speed <= 0.0 {
            continue;
        }

        let current_linear_speed = linear_velocity.dot(*impulse_direction);

        let linear_force = incoming_speed * depth * character_mass.value() * time.delta_secs();
        let linear_acceleration = f32::min(
            incoming_speed - current_linear_speed,
            linear_force * mass.inverse(),
        );

        linear_velocity.0 += impulse_direction * linear_acceleration;

        // Angular push

        let center_of_mass = transform.transform_point(center_of_mass.0);
        let contact_point = hit.point;
        let contact_offset = contact_point - center_of_mass;

        let torque = contact_offset.cross(impulse_direction * linear_force);

        // Get current angular velocity in the torque direction
        let Ok(torque_axis) = Dir3::new(torque) else {
            continue;
        };
        let current_angular_speed = angular_velocity.0.dot(*torque_axis);

        const MAX_TORQUE: f32 = 100.0;

        let angular_acceleration = f32::min(
            f32::max(0.0, incoming_speed - current_angular_speed),
            f32::min(
                MAX_TORQUE * time.delta_secs(),
                (inertia.inverse() * torque).length(),
            ),
        );

        // Apply limited angular change
        angular_velocity.0 += torque_axis * angular_acceleration;
    }
}

/// Triggered when the character becomes grounded during a movement update.
///
/// This is only triggered for the last ground the character touched during the update and will not be triggered
/// if the character was already grounded prior to the start of the update.
#[derive(Event, Deref)]
pub struct OnGroundEnter(pub Ground);

/// Triggered when the character becomes ungrounded during a movement update.
///
/// This is only triggered if the character is ungrounded at the end of the update.
#[derive(Event, Deref)]
pub struct OnGroundLeave(pub Ground);

pub(crate) fn move_character_with_platform(
    mut characters: Query<(&Character, &Grounding, &mut InheritedVelocity, &Transform)>,
    platforms: Query<(
        &LinearVelocity,
        &AngularVelocity,
        &GlobalTransform,
        &ComputedCenterOfMass,
    )>,
    time: Res<Time>,
) -> Result {
    for (character, grounding, mut velocity_on_platform, transform) in &mut characters {
        let Some(platform) = grounding.inner_ground else {
            *velocity_on_platform = InheritedVelocity::default();
            continue;
        };

        let (
            platform_linear_velocity,
            platform_angular_velocity,
            platform_transform,
            platform_center_of_mass,
        ) = platforms.get(platform.entity)?;

        let platform_position = platform_transform.transform_point(platform_center_of_mass.0);

        *velocity_on_platform = InheritedVelocity::at_point(
            platform_position,
            platform_linear_velocity.0,
            platform_angular_velocity.0,
            transform.translation,
            character.velocity,
            time.delta_secs(),
        );
    }

    Ok(())
}

pub(crate) fn inherit_platform_velocity(
    trigger: Trigger<OnGroundLeave>,
    mut query: Query<(&mut Character, &mut InheritedVelocity)>,
) -> Result {
    let (mut character, mut platform_velocity) = query.get_mut(trigger.target())?;
    character.velocity += mem::take(&mut platform_velocity.0);
    Ok(())
}

pub(crate) fn move_character(
    mut commands: Commands,
    spatial_query: SpatialQuery,
    mut query: Query<(
        Entity,
        &mut Character,
        &InheritedVelocity,
        Option<(&mut Grounding, &GroundingSettings)>,
        Option<&SteppingSettings>,
        &mut Transform,
        &Collider,
        &CharacterFilter,
        Has<Sensor>,
        Option<&mut DebugMotion>,
        Has<DebugMode>,
    )>,
    mut bodies: Query<&RigidBody>,
    mut collision_started_events: EventWriter<CollisionStarted>,
    mut collision_ended_events: EventWriter<CollisionEnded>,
    time: Res<Time>,
) -> Result {
    for (
        entity,
        mut character,
        platform_velocity,
        mut grounding,
        stepping_settings,
        mut transform,
        collider,
        filter,
        is_sensor,
        mut debug_motion,
        debug_mode,
    ) in &mut query
    {
        transform.translation += platform_velocity.0 * time.delta_secs();

        if is_sensor {
            transform.translation += character.velocity * time.delta_secs();
            continue;
        }

        // let current_ground_normal = character.grounding.normal();
        let current_ground_normal = grounding.as_ref().and_then(|(g, _)| g.normal());

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

        if let Some((_, grounding_settings)) = grounding.as_ref() {
            assert_eq!(
                None, grounding_settings.layer_mask,
                "custom layer masks for walkable ground is not supported"
            );
        }

        if let Some(stepping_settings) = stepping_settings {
            assert_eq!(
                None, stepping_settings.layer_mask,
                "custom layer masks for steppable surfaces are not supported"
            );
        }

        // When already grounded, add a small epsilon to make sure we don't randomly
        // loose grip of the surface when the ground angle matches the max angle perfectly
        let walkable_angle = |base_angle, is_grounded| match is_grounded {
            true => base_angle + 0.01,
            false => base_angle,
        };

        let mut movement = collide_and_slide(
            collider,
            transform.translation,
            transform.rotation,
            character.velocity,
            current_ground_normal,
            CollideAndSlideConfig {
                max_iterations: character.max_slide_count,
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
                 hit,
                 ..
             }| {
                let surface = match grounding.as_ref() {
                    Some((grounding, grounding_settings)) => Surface::new(
                        hit.normal,
                        walkable_angle(grounding_settings.max_angle, grounding.is_grounded()),
                        character.up,
                    ),
                    None => Surface {
                        normal: Dir3::new(hit.normal).unwrap(),
                        is_walkable: false,
                    },
                };

                // Try to step over obstacles
                if let Some((stepping_settings, (grounding, grounding_settings))) =
                    stepping_settings.zip(grounding.as_ref())
                {
                    let step_condition = match stepping_settings.behaviour {
                        SteppingBehaviour::Never => false,
                        SteppingBehaviour::Grounded => grounding.is_grounded(),
                        SteppingBehaviour::GroundedOrFalling => {
                            grounding.is_grounded() || state.velocity.dot(*character.up) < 0.0
                        }
                        SteppingBehaviour::Always => true,
                    };

                    let is_dynamic = match bodies.get_mut(hit.entity) {
                        Ok(rb) => rb.is_dynamic(),
                        Err(_) => false,
                    };

                    let try_step = !surface.is_walkable && !is_dynamic && step_condition;

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
                                    align_with_surface(state.velocity, hit.normal, *character.up);
                                state.ground = Some(Ground::new(hit.entity, hit.normal));
                                state.offset += offset;

                                return None;
                            }
                        }
                    }
                }

                // Trigger collision events
                collision_started_events.write(CollisionStarted(entity, hit.entity));

                // For now, assume the collision is ended instantly which is probably the case with move and slide anyways
                collision_ended_events.write(CollisionEnded(entity, hit.entity));

                if debug_mode {
                    if let Some(lines) = debug_motion.as_mut() {
                        let duration = duration - state.remaining_time;
                        lines.push(
                            duration,
                            DebugPoint {
                                translation: transform.translation + state.offset,
                                velocity: state.velocity,
                                hit: Some(DebugHit {
                                    point: hit.point,
                                    normal: hit.normal,
                                    is_walkable: surface.is_walkable,
                                }),
                            },
                        );
                    }
                }

                Some(surface)
            },
        );

        let mut new_translation = transform.translation + movement.offset;

        if let Some((grounding, grounding_settings)) = grounding.as_ref() {
            if grounding.is_grounded() || movement.ground.is_some() {
                if let Some((ground, hit)) = ground_check(
                    collider,
                    new_translation,
                    transform.rotation,
                    character.up,
                    grounding_settings.max_distance,
                    character.skin_width,
                    walkable_angle(grounding_settings.max_angle, grounding.is_grounded()),
                    &spatial_query,
                    &filter.0,
                ) {
                    movement.ground = Some(ground);

                    let mut hit_roof = !grounding_settings.snap_to_surface;

                    if grounding_settings.snap_to_surface && hit.distance < 0.0 {
                        hit_roof = sweep(
                            collider,
                            transform.translation,
                            transform.rotation,
                            character.up,
                            -hit.distance,
                            character.skin_width,
                            &spatial_query,
                            &filter.0,
                            true,
                        )
                        .is_some();
                    }

                    if !hit_roof {
                        new_translation -= character.up * hit.distance;
                    }
                } else {
                    movement.ground = None;
                }
            }

            match (grounding.inner_ground, movement.ground) {
                (Some(ground), None) => {
                    commands.entity(entity).trigger(OnGroundLeave(ground));
                }
                (None, Some(ground)) => {
                    commands.entity(entity).trigger(OnGroundEnter(ground));
                }
                _ => {}
            }
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
            if let Some((grounding, _)) = grounding.as_mut() {
                **grounding = Grounding::new(movement.ground);
            }
        }
    }

    Ok(())
}

#[derive(Component, Reflect, Default, Debug)]
#[reflect(Component)]
pub struct InheritedVelocity(pub Vec3);

impl InheritedVelocity {
    pub fn at_point(
        translation: Vec3,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
        point: Vec3,
        relative_velocity: Vec3,
        delta: f32,
    ) -> Self {
        // Calculate an initial estimate of platform velocity at current position
        let initial_velocity =
            inherited_velocity_at_point(translation, linear_velocity, angular_velocity, point);

        // Calculate midpoint position (where we'll be halfway through the timestep)
        let midpoint_position = point + (relative_velocity + initial_velocity) * delta / 2.0;

        // Calculate the final platform velocity
        let velocity = inherited_velocity_at_point(
            translation,
            linear_velocity,
            angular_velocity,
            midpoint_position,
        );

        Self(velocity)
    }
}

pub(crate) fn inherited_velocity_at_point(
    translation: Vec3,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    point: Vec3,
) -> Vec3 {
    let mut inherited_velocity = linear_velocity;

    // Angular velocity component
    if angular_velocity.length_squared() > 0.0 {
        // Vector from platform center to character
        let radius_vector = point - translation;

        // Tangential velocity = angular_velocity × radius_vector
        inherited_velocity += angular_velocity.cross(radius_vector);
    };

    inherited_velocity
}

pub(crate) struct RelativeVelocity(pub Vec3);

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SteppingBehaviour {
    Never,
    Grounded,
    GroundedOrFalling,
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
    InheritedVelocity,
)]
pub struct Character {
    pub velocity: Vec3,
    pub up: Dir3,
    pub skin_width: f32,
    pub max_slide_count: u8,
    pub previous_ground: Option<Ground>,
}

impl Default for Character {
    fn default() -> Self {
        Self {
            velocity: Vec3::ZERO,
            up: Dir3::Y,
            skin_width: 0.02,
            max_slide_count: 8,
            previous_ground: None,
        }
    }
}

pub fn jump(impulse: f32, character: &mut Character, grounding: &mut Grounding) {
    // Remove vertical velocity
    character.velocity = character.velocity.reject_from(*character.up);

    // Push the character away from ramps
    if let Some(ground) = grounding.detach() {
        let impulse = ground.normal * character.velocity.dot(*ground.normal).min(0.0);

        let vertical = impulse.project_onto(*character.up);
        let mut horizontal = impulse - vertical;

        // Reorient horizontal impulse to original velocity direction
        if let Ok(direction) = Dir3::new(character.velocity) {
            horizontal = horizontal.project_onto(*direction);
        }

        character.velocity -= horizontal + vertical;
    }

    character.velocity += character.up * impulse;
}

impl Character {
    // /// Launch the character, clearing the grounded state if launched away from the `ground` normal.
    // pub fn launch(&mut self, impulse: Vec3) {
    //     if let Some(ground) = self.grounding.ground() {
    //         // Clear grounded if launched away from the ground
    //         if ground.normal.dot(impulse) > 0.0 {
    //             self.grounding.detach();
    //         }
    //     }

    //     self.velocity += impulse
    // }

    // /// Launch the character on the `up` axis, overriding the downward velocity.
    // pub fn jump(&mut self, impulse: f32) {
    //     // Remove vertical velocity
    //     self.velocity = self.velocity.reject_from(*self.up);

    //     // Push the character away from ramps
    //     if let Some(ground) = self.grounding.detach() {
    //         let impulse = ground.normal * self.velocity.dot(*ground.normal).min(0.0);

    //         let vertical = impulse.project_onto(*self.up);
    //         let mut horizontal = impulse - vertical;

    //         // Reorient horizontal impulse to original velocity direction
    //         if let Ok(direction) = Dir3::new(self.velocity) {
    //             horizontal = horizontal.project_onto(*direction);
    //         }

    //         self.velocity -= horizontal + vertical;
    //     }

    //     self.velocity += self.up * impulse;
    // }

    // /// Returns `true` if the character is standing on the ground.
    // pub fn is_grounded(&self) -> bool {
    //     self.grounding.is_grounded()
    // }

    pub fn feet_position(&self, shape: &Collider, rotation: Quat) -> Vec3 {
        let aabb = shape.aabb(Vec3::ZERO, rotation);
        let down = aabb.min.dot(*self.up) - self.skin_width;
        self.up * down
    }
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
#[require(MoveInput)]
pub struct CharacterMovement {
    pub target_speed: f32,
    pub acceleration: f32,
}

impl CharacterMovement {
    pub const DEFAULT_GROUND: Self = Self {
        target_speed: 8.0,
        acceleration: 100.0,
    };

    pub const DEFAULT_AIR: Self = Self {
        target_speed: 8.0,
        acceleration: 20.0,
    };
}

#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
pub struct CharacterGravity(pub Vec3);

impl Default for CharacterGravity {
    fn default() -> Self {
        Self(Vec3::Y * -9.81)
    }
}

impl CharacterGravity {
    pub const ZERO: Self = Self(Vec3::ZERO);
}

#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
pub struct CharacterFriction(pub f32);

impl Default for CharacterFriction {
    fn default() -> Self {
        Self(120.0)
    }
}

impl CharacterFriction {
    pub const ZERO: Self = Self(0.0);
}

#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
pub struct CharacterDrag(pub f32);

impl Default for CharacterDrag {
    fn default() -> Self {
        Self(0.01)
    }
}

impl CharacterDrag {
    pub const ZERO: Self = Self(0.0);
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
        self.previous = mem::take(&mut self.value);
        self.previous
    }

    pub fn set(&mut self, value: Vec3) {
        self.value = value;
    }

    pub fn previous(&self) -> Vec3 {
        self.previous
    }
}

fn apply_impulse_on_point(
    impulse: Vec3,
    point: Vec3,
    linear_velocity: &mut Vec3,
    angular_velocity: &mut Vec3,
    inverse_mass: f32,
    inverse_inertia: Mat3,
    center_of_mass: Vec3,
) {
    *linear_velocity += inverse_mass * impulse;

    let torque = (point - center_of_mass).cross(impulse);
    *angular_velocity += inverse_inertia * torque;
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
