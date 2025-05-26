use std::mem;

use avian3d::prelude::*;
use bevy::{
    color::palettes::css::*,
    ecs::{intern::Interned, schedule::ScheduleLabel},
    input::InputSystem,
    prelude::*,
};
use debug::{CharacterGizmos, DebugHit, DebugMode, DebugMotion, DebugPoint};
use ground::{Ground, Grounding, GroundingConfig, ground_check, is_walkable};
use projection::{CollisionState, Surface, align_with_surface, project_velocity};
use sweep::{
    CollideAndSlideConfig, MovementImpact, SweepHitData, collide_and_slide, step_check, sweep,
};

pub mod debug;
pub mod ground;
pub(crate) mod projection;
pub mod sweep;

pub mod prelude {
    pub use crate::{
        Character, CharacterDrag, CharacterFriction, CharacterGravity, CharacterMovement,
        CharacterPlugin, KinematicVelocity, MoveInput, OnGroundEnter, OnGroundLeave, OnStep,
        SteppingBehaviour, SteppingConfig,
        ground::{Grounding, GroundingConfig},
        sweep::CollideAndSlideConfig,
    };
}

#[derive(SystemSet, Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum CharacterSystems {
    Prepare,
    Forces,
    PhysicsInteractions,
    ApplyMovement,
}

pub struct CharacterPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl Default for CharacterPlugin {
    fn default() -> Self {
        Self::new(FixedPostUpdate)
    }
}

impl CharacterPlugin {
    pub fn new<S: ScheduleLabel>(schedule: S) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Plugin for CharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((debug::plugin,));

        app.add_systems(PreUpdate, clear_movement_input.before(InputSystem));

        app.configure_sets(
            self.schedule,
            (
                CharacterSystems::Prepare.before(CharacterSystems::PhysicsInteractions),
                CharacterSystems::PhysicsInteractions.before(PhysicsSet::Prepare),
                CharacterSystems::ApplyMovement.after(PhysicsSet::Sync),
            ),
        );

        app.add_systems(
            self.schedule,
            update_character_filter.in_set(CharacterSystems::Prepare),
        );

        app.add_systems(
            self.schedule,
            (
                character_drag,
                character_friction,
                character_gravity,
                character_acceleration,
            )
                // TODO: this should probably have it's own set
                .in_set(CharacterSystems::Prepare)
                .chain(),
        );

        app.add_systems(
            self.schedule,
            physics_interactions.in_set(CharacterSystems::PhysicsInteractions),
        );

        app.add_systems(
            self.schedule,
            (update_platform_velocity, move_with_platform, move_character)
                .in_set(CharacterSystems::ApplyMovement)
                .chain(),
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
    mut query: Query<(Entity, &mut CollideAndSlideFilter, &CollisionLayers)>,
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
    default_gravity: Res<Gravity>,
    mut query: Query<(
        &mut KinematicVelocity,
        Option<&CharacterGravity>,
        Option<&Grounding>,
        Option<&GravityScale>,
    )>,
    time: Res<Time>,
) {
    for (mut velocity, character_gravity, grounding, gravity_scale) in &mut query {
        if grounding.map_or(false, |g| g.is_grounded()) {
            continue;
        }

        let mut gravity = character_gravity.map(|g| g.0).unwrap_or(default_gravity.0);

        if let Some(gravity_scale) = gravity_scale {
            gravity *= gravity_scale.0;
        }

        velocity.0 += gravity * time.delta_secs();
    }
}

pub(crate) fn character_friction(
    mut characters: Query<(&mut KinematicVelocity, &Grounding, &CharacterFriction)>,
    frictions: Query<&FrictionScale>,
    colliders: Query<&ColliderOf>,
    time: Res<Time>,
) {
    for (mut velocity, grounding, character_friction) in &mut characters {
        let Some(ground) = grounding.ground() else {
            continue;
        };

        let mut friction = character_friction.0;

        // Multiply friction by the friction scale
        if let Ok(s) = frictions.get(ground.entity) {
            friction *= s.0;
        } else if let Ok(collider_of) = colliders.get(ground.entity) {
            if let Ok(s) = frictions.get(collider_of.body) {
                friction *= s.0;
            }
        }

        let f = friction_factor(velocity.0, friction, time.delta_secs());

        velocity.0 *= f;
    }
}

pub(crate) fn character_drag(
    mut query: Query<(&mut KinematicVelocity, &CharacterDrag)>,
    time: Res<Time>,
) {
    for (mut velocity, drag) in &mut query {
        velocity.0 *= drag_factor(drag.0, time.delta_secs());
    }
}

pub(crate) fn character_acceleration(
    mut query: Query<(
        &MoveInput,
        &mut KinematicVelocity,
        Option<(&Grounding, &GroundingConfig)>,
        &CharacterMovement,
        Has<DebugMode>,
    )>,
    time: Res<Time>,
) {
    for (move_input, mut character_velocity, grounding, movement, debug_mode) in &mut query {
        let Ok((direction, throttle)) = Dir3::new_and_length(move_input.value) else {
            continue;
        };

        if debug_mode {
            character_velocity.0 = direction * movement.target_speed * throttle;
            continue;
        }

        let mut direction = *direction;
        let mut velocity = character_velocity.0;

        if let Some((grounding, grounding_settings)) = grounding {
            if let Some(normal) = grounding.normal() {
                direction = align_with_surface(direction, *normal, *grounding_settings.up);
                velocity = align_with_surface(velocity, *normal, *grounding_settings.up);
            }
        }

        let move_accel = acceleration(
            velocity,
            direction,
            movement.acceleration * throttle,
            movement.target_speed * throttle,
            time.delta_secs(),
        );

        character_velocity.0 += move_accel;
    }
}

pub(crate) fn depenetrate_character(
    mut commands: Commands,
    mut overlaps: Local<Vec<(Dir3, f32)>>,
    mut gizmos: Gizmos<CharacterGizmos>,
    collisions: Collisions,
    mut query: Query<(
        Entity,
        &mut KinematicVelocity,
        Option<(&mut Grounding, &GroundingConfig)>,
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

        let (entity, mut velocity, mut grounding, mut transform) =
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
                    let surface = Surface::new(
                        hit_normal,
                        grounding_settings.max_angle,
                        grounding_settings.up,
                    );
                    let ground_normal = grounding.normal();
                    let obstruction_normal = surface
                        .obstruction_normal(ground_normal, grounding_settings.up)
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

            if velocity.0.dot(hit_normal) > 0.0 {
                continue;
            }

            match grounding {
                Some((_, grounding_settings)) => {
                    velocity.0 = project_velocity(
                        velocity.0,
                        *obstruction_normal,
                        surface.is_walkable,
                        ground_normal,
                        grounding_settings.up,
                    );
                }
                None => {
                    velocity.0 = velocity.0.reject_from(*obstruction_normal);
                }
            }

            if surface.is_walkable {
                if let Some((grounding, _)) = grounding.as_mut() {
                    let ground = Ground::new(other, hit_normal);

                    if !grounding.is_grounded() {
                        commands.entity(entity).trigger(OnGroundEnter(ground));
                    }

                    **grounding = ground.into();
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

/// Push dynamic bodies before movement and physics simulation.
fn physics_interactions(
    spatial_query: SpatialQuery,
    mut characters: Query<
        (
            &CollideAndSlideConfig,
            &mut KinematicVelocity,
            Option<(&mut Grounding, &GroundingConfig)>,
            &Transform,
            &Collider,
            &ComputedMass,
            &CollideAndSlideFilter,
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
    )>,
    time: Res<Time>,
) {
    for (
        config,
        mut character_velocity,
        grounding,
        character_transform,
        character_collider,
        character_mass,
        filter,
    ) in &mut characters
    {
        let Ok((sweep_direction, target_sweep_distance)) =
            Dir3::new_and_length(character_velocity.0 * time.delta_secs())
        else {
            continue;
        };

        let Some(hit) = sweep(
            character_collider,
            character_transform.translation,
            character_transform.rotation,
            sweep_direction,
            target_sweep_distance,
            config.skin_width,
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
        )) = bodies.get_mut(hit.entity)
        else {
            continue;
        };

        if !rb.is_dynamic() {
            continue;
        }

        // Don't push the body the character is standing on
        if let Some((mut grounding, grounding_config)) = grounding {
            let surface = Surface::new(hit.normal, grounding_config.max_angle, grounding_config.up);

            // Project velocity and set ground
            if surface.is_walkable {
                character_velocity.0 = surface.project_velocity(
                    character_velocity.0,
                    grounding.normal(),
                    grounding_config.up,
                );

                grounding.inner_ground = Some(Ground {
                    entity: hit.entity,
                    normal: surface.normal,
                });
            }

            if grounding.entity() == Some(hit.entity) {
                continue;
            }
        }

        // We have to limit the impulse applied to bodies so that small bodies don't glitch out of existence

        let depth = 1.0 - hit.distance / target_sweep_distance;

        let Ok(impulse_direction) = Dir3::new(-hit.normal) else {
            continue;
        };

        let incoming_speed = character_velocity.0.dot(*impulse_direction);

        if incoming_speed <= 0.0 {
            continue;
        }

        character_velocity.0 -= impulse_direction * incoming_speed * depth;

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

/// Triggered when a character stepped over an obstacle.
#[derive(Event)]
pub struct OnStep {
    /// The translation of the character before stepping.
    pub origin: Vec3,
    /// The movement of the character during the step.
    pub offset: Vec3,
    pub hit: SweepHitData,
}

pub(crate) fn update_platform_velocity(
    mut characters: Query<(
        &KinematicVelocity,
        &Grounding,
        &mut InheritedVelocity,
        &Transform,
    )>,
    platforms: Query<(
        &LinearVelocity,
        &AngularVelocity,
        &GlobalTransform,
        &ComputedCenterOfMass,
    )>,
    time: Res<Time>,
) -> Result {
    for (velocity, grounding, mut velocity_on_platform, transform) in &mut characters {
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
            velocity.0,
            time.delta_secs(),
        );
    }

    Ok(())
}

pub(crate) fn inherit_platform_velocity(
    trigger: Trigger<OnGroundLeave>,
    mut query: Query<(&mut KinematicVelocity, &mut InheritedVelocity)>,
) {
    let Ok((mut velocity, mut platform_velocity)) = query.get_mut(trigger.target()) else {
        return;
    };
    velocity.0 += mem::take(&mut platform_velocity.0);
}

pub(crate) fn move_with_platform(
    mut query: Query<(&mut Transform, &InheritedVelocity)>,
    time: Res<Time>,
) {
    for (mut transform, inherited_velocity) in &mut query {
        transform.translation += inherited_velocity.0 * time.delta_secs();
    }
}

pub(crate) fn move_character(
    mut commands: Commands,
    spatial_query: SpatialQuery,
    mut query: Query<(
        Entity,
        &CollideAndSlideConfig,
        &mut KinematicVelocity,
        Option<(&mut Grounding, &GroundingConfig)>,
        Option<&SteppingConfig>,
        &mut Transform,
        &Collider,
        &CollideAndSlideFilter,
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
        config,
        mut velocity,
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
        if is_sensor {
            transform.translation += velocity.0 * time.delta_secs();
            continue;
        }

        // let current_ground_normal = character.grounding.normal();
        let current_ground_normal = grounding.as_ref().and_then(|(g, _)| g.normal());

        if debug_mode {
            if let Some(lines) = debug_motion.as_mut() {
                let mut point = transform.translation;

                if let Some((_, grounding_settings)) = grounding {
                    point += feet_position(
                        collider,
                        transform.rotation,
                        grounding_settings.up,
                        config.skin_width,
                    );
                }

                lines.push(
                    0.0,
                    DebugPoint {
                        translation: transform.translation,
                        velocity: velocity.0,
                        hit: current_ground_normal.map(|normal| DebugHit {
                            point,
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

        let mut did_step = false;

        let mut movement = collide_and_slide(
            collider,
            transform.translation,
            transform.rotation,
            velocity.0,
            current_ground_normal,
            *config,
            &filter.0,
            &spatial_query,
            duration,
            |velocity, surface| match grounding {
                Some((_, grounding_settings)) => {
                    surface.project_velocity(velocity, current_ground_normal, grounding_settings.up)
                }
                None => velocity.reject_from(*surface.normal),
            },
            |state,
             MovementImpact {
                 end,
                 remaining_motion,
                 hit,
                 ..
             }| {
                let surface = match grounding.as_ref() {
                    Some((grounding, grounding_settings)) => Surface::new(
                        hit.normal,
                        walkable_angle(grounding_settings.max_angle, grounding.is_grounded()),
                        grounding_settings.up,
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
                            grounding.is_grounded()
                                || state.velocity.dot(*grounding_settings.up) < 0.0
                        }
                        SteppingBehaviour::Always => true,
                    };

                    let is_dynamic = match bodies.get_mut(hit.entity) {
                        Ok(rb) => rb.is_dynamic(),
                        Err(_) => false,
                    };

                    let try_step = !surface.is_walkable && !is_dynamic && step_condition;

                    if try_step {
                        if let Ok(direction) =
                            Dir3::new(velocity.0.reject_from(*grounding_settings.up))
                        {
                            if let Some((offset, hit)) = step_check(
                                collider,
                                end,
                                transform.rotation,
                                direction,
                                remaining_motion,
                                grounding_settings.up,
                                grounding_settings.max_angle,
                                1.0,
                                0.1,
                                stepping_settings.max_height,
                                config.skin_width,
                                &spatial_query,
                                &filter.0,
                            ) {
                                commands.entity(entity).trigger(OnStep {
                                    origin: transform.translation + state.offset,
                                    offset,
                                    hit,
                                });

                                state.velocity = align_with_surface(
                                    state.velocity,
                                    hit.normal,
                                    *grounding_settings.up,
                                );
                                state.ground = Some(Ground::new(hit.entity, hit.normal));
                                state.offset += offset;

                                did_step = true;

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
                    grounding_settings.up,
                    grounding_settings.max_distance,
                    config.skin_width,
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
                            grounding_settings.up,
                            -hit.distance,
                            config.skin_width,
                            &spatial_query,
                            &filter.0,
                            true,
                        )
                        .is_some();
                    }

                    if !hit_roof {
                        new_translation -= grounding_settings.up * hit.distance;
                    }
                } else {
                    movement.ground = None;
                }
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

            let mut point = new_translation;

            if let Some((_, grounding_settings)) = grounding {
                point += feet_position(
                    collider,
                    transform.rotation,
                    grounding_settings.up,
                    config.skin_width,
                );
            }

            lines.push(
                movement.remaining_time,
                DebugPoint {
                    translation: new_translation,
                    velocity: movement.velocity,
                    hit: movement.ground.map(|ground| DebugHit {
                        point,
                        normal: *ground.normal,
                        is_walkable: true,
                    }),
                },
            );
        }

        if !debug_mode {
            if let Some((grounding, grounding_config)) = grounding.as_mut() {
                match (grounding.inner_ground, movement.ground) {
                    (Some(ground), None) => {
                        commands.entity(entity).trigger(OnGroundLeave(ground));
                    }
                    (None, Some(ground)) => {
                        commands.entity(entity).trigger(OnGroundEnter(ground));
                    }
                    _ => {}
                }

                if let Some(ground) = movement.ground {
                    // Make sure the character is not launched up after stepping.
                    // FIXME: doesn't really work
                    if did_step {
                        movement.velocity = align_with_surface(
                            movement.velocity,
                            *ground.normal,
                            *grounding_config.up,
                        );
                    }
                } else if grounding.is_grounded()
                    && movement.velocity.dot(*grounding_config.up) < 0.0
                {
                    movement.velocity = align_with_surface(
                        movement.velocity,
                        *grounding_config.up,
                        *grounding_config.up,
                    );
                }

                **grounding = Grounding::new(movement.ground);
            }

            transform.translation = new_translation;
            velocity.0 = movement.velocity;
        }
    }

    Ok(())
}

/// The velocity of the ground a character is standing ong.
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

/// The velocity of a character.
#[derive(Component, Reflect, Debug, Default, Clone, Copy, Deref, DerefMut)]
#[reflect(Component)]
pub struct KinematicVelocity(pub Vec3);

#[derive(Reflect, Debug, PartialEq, Eq, Clone, Copy)]
pub enum SteppingBehaviour {
    Never,
    Grounded,
    GroundedOrFalling,
    Always,
}

/// Configure stepping for a character.
#[derive(Component, Reflect, Debug, PartialEq, Clone, Copy)]
#[reflect(Component, Default)]
#[require(GroundingConfig)]
pub struct SteppingConfig {
    pub layer_mask: Option<LayerMask>,
    pub max_height: f32,
    pub behaviour: SteppingBehaviour,
}

impl Default for SteppingConfig {
    fn default() -> Self {
        Self {
            layer_mask: None,
            max_height: 0.25,
            behaviour: SteppingBehaviour::Grounded,
        }
    }
}

#[derive(Component, Reflect, Default, Debug, Clone, Copy)]
#[reflect(Component, Default)]
#[require(
    RigidBody = RigidBody::Kinematic,
    Collider = Capsule3d::new(0.4, 1.0),
    CollideAndSlideConfig,
    CollideAndSlideFilter,
    KinematicVelocity,
    InheritedVelocity,
    Grounding,
    GroundingConfig,
    CharacterFriction,
    MoveInput,
)]
pub struct Character;

/// Used for accelerating a character based on it's [`MoveInput`].
#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
#[require(MoveInput, KinematicVelocity)]
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

/// The gravity force affecting a character while it's not grounded.
/// If no gravity is defined then the [`Gravity`] resource will be used instead.
#[derive(Component, Reflect, Debug, Clone, Deref, DerefMut)]
#[reflect(Component, Default)]
#[require(KinematicVelocity)]
pub struct CharacterGravity(pub Vec3);

impl Default for CharacterGravity {
    fn default() -> Self {
        Self(Vec3::Y * -9.81)
    }
}

impl CharacterGravity {
    pub const ZERO: Self = Self(Vec3::ZERO);
}

/// The friction scale when a character walks on an entity.
#[derive(Component, Reflect, Default, Debug, Deref, DerefMut)]
#[reflect(Component)]
pub struct FrictionScale(pub f32);

/// The friction applied to [`KinematicVelocity`] when a character is grounded.
/// Multiplied by the [`FrictionScale`] of the ground entity.
#[derive(Component, Reflect, Debug, Clone, Deref, DerefMut)]
#[reflect(Component)]
#[require(KinematicVelocity)]
pub struct CharacterFriction(pub f32);

impl Default for CharacterFriction {
    fn default() -> Self {
        Self(60.0)
    }
}

impl CharacterFriction {
    pub const ZERO: Self = Self(0.0);
}

/// The drag force applied to the [`KinematicVelocity`] of a character.
#[derive(Component, Reflect, Debug, Clone, Deref, DerefMut)]
#[reflect(Component)]
#[require(KinematicVelocity)]
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
#[derive(Component, Reflect, Default, Debug)]
#[reflect(Component)]
pub struct CollideAndSlideFilter(pub(crate) SpatialQueryFilter);

/// The desired movement direction of a character.
/// The magnitude of the value will be used to scale the acceleration and target speed when [`CharacterMovement`] is used.
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

pub fn jump(impulse: f32, velocity: &mut KinematicVelocity, grounding: &mut Grounding, up: Dir3) {
    // Remove vertical velocity
    velocity.0 = velocity.0.reject_from(*up);

    // Push the character away from ramps
    if let Some(ground) = grounding.detach() {
        let impulse = ground.normal * velocity.0.dot(*ground.normal).min(0.0);

        let vertical = impulse.project_onto(*up);
        let mut horizontal = impulse - vertical;

        // Reorient horizontal impulse to original velocity direction
        if let Ok(direction) = Dir3::new(velocity.0) {
            horizontal = horizontal.project_onto(*direction);
        }

        velocity.0 -= horizontal + vertical;
    }

    velocity.0 += up * impulse;
}

pub fn feet_position(shape: &Collider, rotation: Quat, up: Dir3, skin_width: f32) -> Vec3 {
    let aabb = shape.aabb(Vec3::ZERO, rotation);
    let down = aabb.min.dot(*up) - skin_width;
    up * down
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
