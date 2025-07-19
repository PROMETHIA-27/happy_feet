use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    collide_and_slide::{
        CollideAndSlideConfig, CollideAndSlideFilter, CollisionResponse, MovementHitData,
        MovementState, add_to_filter_on_insert_collider, collide_and_slide,
        init_filter_mask_on_insert_collision_layers, remove_from_filter_on_replace_collider,
    },
    grounding::{Ground, Grounding, GroundingConfig, GroundingState, is_walkable, walkable_angle},
    moving_platform::InheritedVelocity,
    projection::{Surface, align_with_surface},
    stepping::{StepOutput, SteppingBehaviour, SteppingConfig, perform_step},
    sweep::SweepHitData,
};

#[derive(SystemSet, Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct CharacterSystems;

// TODO: the name is misleading since it supports moving other things as well
pub struct CharacterPlugin;

impl Plugin for CharacterPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CollideAndSlideConfig>();

        app.add_observer(init_filter_mask_on_insert_collision_layers);
        app.add_observer(add_to_filter_on_insert_collider);
        app.add_observer(remove_from_filter_on_replace_collider);

        app.configure_sets(
            PhysicsSchedule,
            CharacterSystems.in_set(NarrowPhaseSet::Last),
        );

        app.add_systems(
            PhysicsSchedule,
            (update_query_pipeline, process_movement)
                .chain()
                .in_set(CharacterSystems),
        );
    }
}

/// A component for setting up character movement with grounding and stepping behavior.
#[derive(Component, Reflect, Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component, Default, Debug, Clone)]
#[require(
    RigidBody = RigidBody::Kinematic,
    KinematicVelocity,
    InheritedVelocity,
    GroundingConfig,
    SteppingConfig,
    CollisionEventsEnabled,
)]
pub struct Character;

/// The actual movement during the last [`collide-and-slide`](collide_and_slide) update.
#[derive(Component, Reflect, Deref, Debug, Default, Clone, Copy)]
#[reflect(Component, Debug, Default, Clone)]
#[component(immutable)]
pub struct MovementDelta(pub Vec3);

/// The velocity of a kinematic body that is moved using [`collide-and-slide`](collide_and_slide).
#[derive(Component, Reflect, Deref, DerefMut, Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component, Debug, Default, Clone)]
#[require(CollideAndSlideFilter)]
pub struct KinematicVelocity(pub Vec3);

/// Event that is triggered when a character collides with an obstacle during movement.
#[derive(Event, Reflect)]
pub struct OnHit {
    /// The velocity of the entity at the moment of impact
    pub velocity: Vec3,
    /// Detailed information about the collision, including the hit entity, position, and normal
    pub hit: MovementHitData,
}

/// Triggered when a character stepped over an obstacle.
#[derive(Event, Reflect)]
pub struct OnStep {
    /// The velocity of the entity before stepping
    pub velocity: Vec3,
    /// The translation of the character before stepping.
    pub origin: Vec3,
    /// The movement of the character during the step.
    pub offset: Vec3,
    pub hit: SweepHitData,
}

// TODO: is this necessary?
fn update_query_pipeline(mut spatial_query: SpatialQuery) {
    spatial_query.update_pipeline();
}

#[allow(clippy::type_complexity)]
fn process_movement(
    // mut gizmos: Gizmos,
    global_collide_and_slide_config: Res<CollideAndSlideConfig>,
    query_pipeline: Res<SpatialQueryPipeline>,
    mut commands: Commands,
    mut query: Query<(
        Entity,
        &mut KinematicVelocity,
        &mut Position,
        &Rotation,
        &Collider,
        &CollideAndSlideFilter,
        Option<&CollideAndSlideConfig>,
        Option<(&mut Grounding, &GroundingConfig, &mut GroundingState)>,
        Option<&SteppingConfig>,
        Has<CollisionEventsEnabled>,
        Has<Sensor>,
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
        collide_and_slide_config,
        grounding,
        stepping,
        collision_events_enabled,
        is_sensor,
    ) in &mut query
    {
        // Sensors don't need to collide
        // TODO: should we still trigger OnHit events maybe?
        if is_sensor {
            position.0 += velocity.0 * time.delta_secs();
            continue;
        }

        let collide_and_slide_config = collide_and_slide_config
            .copied()
            .unwrap_or(*global_collide_and_slide_config);

        // Filter out sensor entities from collision detection
        let filter_hits = |hit: &SweepHitData| !sensors.contains(hit.entity);

        let is_grounded = grounding.as_ref().is_some_and(|(g, ..)| g.is_grounded());

        let mut movement = MovementState::new(velocity.0, position.0, time.delta_secs());

        collide_and_slide(
            &mut movement,
            collider,
            rotation.0,
            is_grounded,
            &collide_and_slide_config,
            &query_pipeline,
            &filter.0,
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
                            movement.position(),
                            rotation.0,
                            horizontal_direction,
                            horizontal_motion,
                            grounding_config.up_direction,
                            collide_and_slide_config.skin_width,
                            &query_pipeline,
                            &filter.0,
                            filter_hits,
                            |hit| {
                                // Only step on surfaces that are walkable
                                if !is_walkable(
                                    hit.normal,
                                    // Not sure if this is necessary ???
                                    grounding_config.max_angle - 0.01,
                                    *grounding_config.up_direction,
                                ) {
                                    return false;
                                }

                                // Stepping on dynamic bodies is a bit buggy right now ):
                                if let Ok(rb) = rigid_bodies.get(hit.entity)
                                    && rb.is_dynamic()
                                {
                                    return false;
                                }

                                true
                            },
                        )
                    {
                        let offset = grounding_config.up_direction * vertical
                            + horizontal_direction * horizontal;
                        let duration = horizontal * time.delta_secs();

                        // Trigger step event
                        commands.entity(entity).trigger(OnStep {
                            origin: movement.position(),
                            velocity: movement.velocity,
                            offset,
                            hit: step_hit,
                        });

                        // Update movement state
                        movement.offset += offset;
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

                // Trigger hit event
                commands.entity(entity).trigger(OnHit {
                    hit,
                    velocity: movement.velocity,
                });

                // Write collision events
                if collision_events_enabled {
                    collision_started_events.write(CollisionStarted(entity, hit.entity));
                    // Assume the collision is ended immediately, which it did because we slided (:
                    collision_ended_events.write(CollisionEnded(entity, hit.entity));
                }

                CollisionResponse::Slide
            },
            |velocity, surface| match grounding.as_ref() {
                Some((grounding, config, _)) => {
                    surface.project_velocity(velocity, grounding.normal(), config.up_direction)
                }
                None => velocity.reject_from(*surface.normal),
            },
        );

        commands
            .entity(entity)
            .insert(MovementDelta(movement.offset));

        // Apply movement
        position.0 += movement.offset;
        velocity.0 = movement.velocity;

        if let Some((_, _, mut grounding_state)) = grounding {
            grounding_state.pending = movement.ground;
        }
    }
}
