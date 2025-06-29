use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    collide_and_slide::{
        CollideAndSlideConfig, CollideAndSlideFilter, CollideAndSlidePlugin, CollisionResponse,
        MovementHitData, MovementState, collide_and_slide,
    },
    grounding::{Ground, Grounding, GroundingConfig, PreviousGrounding, ground_check, is_walkable},
    moving_platform::InheritedVelocity,
    projection::{Surface, align_with_surface, project_velocity},
    stepping::{StepOutput, SteppingBehaviour, SteppingConfig, perform_step},
    sweep::{SweepHitData, sweep_filtered},
};

// TODO:
// - the grounding stuff should be in grounding.rs
// - depenetrate should be in depenetrate.rs

fn walkable_angle(max_angle: f32, is_grounded: bool) -> f32 {
    match is_grounded {
        true => max_angle + 0.01,
        false => max_angle,
    }
}

#[derive(SystemSet, Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct CharacterSystems;

pub struct CharacterPlugin;

impl Plugin for CharacterPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(
            PhysicsSchedule,
            CharacterSystems.in_set(NarrowPhaseSet::Last),
        );

        app.add_systems(
            PhysicsSchedule,
            (
                depenetrate,
                update_query_pipeline,
                process_movement,
                detect_ground,
                trigger_grounding_events,
            )
                .chain()
                .in_set(CharacterSystems),
        );
    }

    fn finish(&self, app: &mut App) {
        // Requires the CollideAndSlidePlugin to function
        if !app.is_plugin_added::<CollideAndSlidePlugin>() {
            app.add_plugins(CollideAndSlidePlugin);
        }
    }
}

#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component, Debug, Clone)]
#[require(
    RigidBody = RigidBody::Kinematic,
    KinematicVelocity,
    CollideAndSlideConfig,
)]
pub struct Projectile;

#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component, Debug, Clone)]
#[require(
    RigidBody = RigidBody::Kinematic,
    KinematicVelocity,
    InheritedVelocity,
    CollideAndSlideConfig,
    GroundingConfig,
    SteppingConfig,
)]
pub struct Character;

/// The velocity of a character.
#[derive(Component, Reflect, Debug, Default, Clone, Copy, Deref, DerefMut)]
#[reflect(Component, Debug, Default, Clone)]
pub struct KinematicVelocity(pub Vec3);

#[derive(Event, Reflect)]
pub struct CharacterHit {
    pub velocity: Vec3,
    pub hit: MovementHitData,
}

/// Triggered when a character stepped over an obstacle.
#[derive(Event, Reflect)]
pub struct CharacterStep {
    /// The translation of the character before stepping.
    pub origin: Vec3,
    /// The movement of the character during the step.
    pub offset: Vec3,
    pub hit: SweepHitData,
}

/// Triggered when the character becomes grounded during a movement update.
///
/// This is only triggered for the last ground the character touched during the update and will not be triggered
/// if the character was already grounded prior to the start of the update.
#[derive(Event, Reflect, Deref)]
pub struct GroundEnter(pub Ground);

/// Triggered when the character becomes ungrounded during a movement update.
///
/// This is only triggered if the character is ungrounded at the end of the update.
#[derive(Event, Reflect, Deref)]
pub struct GroundLeave(pub Ground);

// TODO: is this necessary?
fn update_query_pipeline(mut spatial_query: SpatialQuery) {
    spatial_query.update_pipeline();
}

#[allow(clippy::type_complexity)]
fn process_movement(
    query_pipeline: Res<SpatialQueryPipeline>,
    mut commands: Commands,
    mut query: Query<(
        Entity,
        &mut KinematicVelocity,
        &mut Position,
        &Rotation,
        &Collider,
        &CollideAndSlideFilter,
        &CollideAndSlideConfig,
        Option<(&mut Grounding, &GroundingConfig, &mut PreviousGrounding)>,
        Option<&SteppingConfig>,
    )>,
    rigid_bodies: Query<&RigidBody>,
    sensors: Query<Entity, With<Sensor>>,
    time: Res<Time>,
    mut collision_started_events: EventWriter<CollisionStarted>,
    mut collision_ended_events: EventWriter<CollisionEnded>,
) {
    for (
        entity,
        mut velocity,
        mut position,
        rotation,
        collider,
        filter,
        config,
        grounding,
        stepping,
    ) in &mut query
    {
        let is_grounded = grounding.as_ref().is_some_and(|(g, ..)| g.is_grounded());

        // Ignore sensors
        let filter_hits = |hit: &SweepHitData| !sensors.contains(hit.entity);

        let mut movement = MovementState::new(velocity.0, position.0, time.delta_secs());

        collide_and_slide(
            &mut movement,
            collider,
            rotation.0,
            is_grounded,
            config,
            &filter.0,
            &query_pipeline,
            |hit| {
                if !filter_hits(hit) {
                    return None;
                }

                Some(match grounding.as_ref() {
                    Some((grounding, grounding_config, _)) => Surface::new(
                        hit.normal,
                        walkable_angle(grounding_config.max_angle, grounding.is_grounded()),
                        grounding_config.up_direction,
                    ),
                    None => Surface {
                        normal: Dir3::new(hit.normal).unwrap(),
                        is_walkable: false,
                    },
                })
            },
            |movement, hit| {
                // Stepping logic
                if !hit.surface.is_walkable
                    && let Some((stepping_config, (grounding, grounding_config, _))) =
                        stepping.zip(grounding.as_ref())
                    && match stepping_config.behaviour {
                        SteppingBehaviour::Never => false,
                        SteppingBehaviour::Grounded => grounding.is_grounded(),
                        SteppingBehaviour::Always => true,
                    }
                {
                    let remaining_horizontal_velocity = (movement.velocity
                        * movement.remaining_time)
                        .reject_from(*grounding_config.up_direction);

                    if let Ok((horizontal_direction, horizontal_motion)) =
                        Dir3::new_and_length(remaining_horizontal_velocity)
                        && let Some(StepOutput {
                            horizontal,
                            vertical,
                            hit: step_hit,
                        }) = perform_step(
                            stepping_config,
                            collider,
                            movement.position,
                            rotation.0,
                            horizontal_direction,
                            horizontal_motion,
                            grounding_config.up_direction,
                            config.skin_width,
                            &query_pipeline,
                            &filter.0,
                            filter_hits,
                            |hit| {
                                if is_walkable(
                                    hit.normal,
                                    grounding_config.max_angle - 0.01,
                                    *grounding_config.up_direction,
                                ) && let Ok(rb) = rigid_bodies.get(hit.entity)
                                    && !rb.is_dynamic()
                                {
                                    true
                                } else {
                                    false
                                }
                            },
                        )
                    {
                        let offset = grounding_config.up_direction * vertical
                            + horizontal_direction * horizontal;
                        let duration = horizontal * time.delta_secs();

                        // Trigger step event
                        commands.entity(entity).trigger(CharacterStep {
                            origin: movement.position,
                            offset,
                            hit: step_hit,
                        });

                        // Update movement state
                        movement.position += offset;
                        movement.velocity = align_with_surface(
                            movement.velocity,
                            step_hit.normal,
                            *grounding_config.up_direction,
                        );
                        movement.ground = Some(Ground::new(step_hit.entity, step_hit.normal));
                        movement.remaining_time = (movement.remaining_time - duration).max(0.0);

                        // Obstruction was avoided, skip projecting velocity
                        return CollisionResponse::Skip;
                    }
                }

                commands.entity(entity).trigger(CharacterHit {
                    hit,
                    velocity: movement.velocity,
                });

                // Write collision events
                collision_started_events.write(CollisionStarted(entity, hit.entity));
                collision_ended_events.write(CollisionEnded(entity, hit.entity)); // Assume the collision is ended immediately

                CollisionResponse::Slide
            },
            |velocity, surface| match grounding.as_ref() {
                Some((grounding, config, _)) => {
                    surface.project_velocity(velocity, grounding.normal(), config.up_direction)
                }
                None => velocity.reject_from(*surface.normal),
            },
        );

        // Apply movement

        position.0 = movement.position;
        velocity.0 = movement.velocity;

        if let Some((mut grounding, _, mut previous_grounding)) = grounding {
            previous_grounding.0 = *grounding;
            *grounding = Grounding::new(movement.ground);
        }
    }
}

fn detect_ground(
    query_pipeline: Res<SpatialQueryPipeline>,
    mut query: Query<(
        &mut Position,
        &Rotation,
        &KinematicVelocity,
        &PreviousGrounding,
        &mut Grounding,
        &GroundingConfig,
        &Collider,
        &CollideAndSlideConfig,
        &CollideAndSlideFilter,
    )>,
    sensors: Query<Entity, With<Sensor>>,
) {
    for (
        mut position,
        rotation,
        velocity,
        previous_grounding,
        mut grounding,
        grounding_config,
        collider,
        config,
        filter,
    ) in &mut query
    {
        if !previous_grounding.is_grounded() && !grounding.is_grounded() {
            continue;
        }

        // Ignore sensors
        let filter_hits = |hit: &SweepHitData| !sensors.contains(hit.entity);

        // Check for ground
        let Some((ground, hit)) = ground_check(
            collider,
            position.0,
            rotation.0,
            Dir3::new(velocity.0).ok(),
            grounding_config.up_direction,
            grounding_config.max_distance,
            config.skin_width,
            grounding_config.max_angle,
            &query_pipeline,
            &filter.0,
            filter_hits,
        ) else {
            grounding.inner_ground = None;
            continue;
        };

        // Snap to the ground
        if grounding_config.snap_to_surface
            // && hit.distance > 0.0
            && sweep_filtered(
            collider,
            position.0,
            rotation.0,
            grounding_config.up_direction,
            -hit.distance,
            config.skin_width,
            &query_pipeline,
            &filter.0,
            true,
            filter_hits,
        )
            .is_none()
        {
            position.0 -= grounding_config.up_direction * hit.distance;
        }

        grounding.inner_ground = Some(ground);
    }
}

fn trigger_grounding_events(
    mut query: Query<(Entity, &Grounding, &PreviousGrounding)>,
    mut commands: Commands,
) {
    for (entity, grounding, previous_grounding) in &mut query {
        match (previous_grounding.inner_ground, grounding.inner_ground) {
            (Some(ground), None) => {
                commands.entity(entity).trigger(GroundLeave(ground));
            }
            (None, Some(ground)) => {
                commands.entity(entity).trigger(GroundEnter(ground));
            }
            _ => {}
        }
    }
}

pub(crate) fn depenetrate(
    mut commands: Commands,
    mut overlaps: Local<Vec<(Dir3, f32)>>,
    collisions: Collisions,
    mut query: Query<(
        Entity,
        &mut Position,
        &mut KinematicVelocity,
        Option<(&mut Grounding, &GroundingConfig)>,
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

        let (entity, mut position, mut velocity, mut grounding) =
            if let Ok(character) = query.get_mut(rb1) {
                other = rb2;
                character
            } else if let Ok(character) = query.get_mut(rb2) {
                other = rb1;
                character
            } else {
                continue;
            };

        // TODO: crease / corner handling (?)

        for manifold in &contacts.manifolds {
            let hit_normal = match entity == rb1 {
                true => -manifold.normal,
                false => manifold.normal,
            };

            let (surface, obstruction_normal, ground_normal) = match grounding.as_ref() {
                Some((grounding, grounding_config)) => {
                    let surface = Surface::new(
                        hit_normal,
                        grounding_config.max_angle,
                        grounding_config.up_direction,
                    );
                    let ground_normal = grounding.normal();
                    let obstruction_normal = surface
                        .obstruction_normal(ground_normal, grounding_config.up_direction)
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
                Some((_, grounding_config)) => {
                    velocity.0 = project_velocity(
                        velocity.0,
                        *obstruction_normal,
                        surface.is_walkable,
                        ground_normal,
                        grounding_config.up_direction,
                    );
                }
                None => {
                    velocity.0 = velocity.0.reject_from(*obstruction_normal);
                }
            }

            if surface.is_walkable
                && let Some((grounding, _)) = grounding.as_mut()
            {
                let ground = Ground::new(other, hit_normal);

                if !grounding.is_grounded() {
                    commands.entity(entity).trigger(GroundEnter(ground));
                }

                **grounding = ground.into();
            }
        }

        // TODO: This is probably not necessary
        for i in 0..overlaps.len() {
            let (direction, depth) = overlaps[i];

            if depth <= 0.0 {
                continue;
            }

            position.0 += direction * depth;

            for j in i..overlaps.len() {
                let (next_direction, ref mut next_depth) = overlaps[j];

                let fixed = f32::max(0.0, direction.dot(*next_direction) * depth);
                *next_depth -= fixed;
            }
        }
    }
}
