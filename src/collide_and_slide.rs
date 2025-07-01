use std::mem;

use avian3d::prelude::*;
use bevy::{ecs::relationship::RelationshipSourceCollection, prelude::*};

use crate::{
    grounding::Ground,
    projection::{CollisionState, Surface},
    sweep::{SweepHitData, sweep},
};

pub struct CollideAndSlidePlugin;

impl Plugin for CollideAndSlidePlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(init_filter_mask_on_insert_collision_layers);
        app.add_observer(add_to_filter_on_insert_collider);
        app.add_observer(remove_from_filter_on_replace_collider);
    }
}

pub fn collide_and_slide(
    state: &mut MovementState,
    shape: &Collider,
    rotation: Quat,
    is_grounded: bool,
    config: &CollideAndSlideConfig,
    filter: &SpatialQueryFilter,
    query_pipeline: &SpatialQueryPipeline,
    mut get_surface: impl FnMut(&SweepHitData) -> Option<Surface>,
    mut on_hit: impl FnMut(&mut MovementState, MovementHitData) -> CollisionResponse,
    mut project_velocity: impl FnMut(Vec3, Surface) -> Vec3,
) {
    let mut collision_state = CollisionState::default();
    let mut previous_velocity = state.velocity;

    for _ in 0..config.max_iterations {
        let Ok((direction, max_distance)) =
            Dir3::new_and_length(state.velocity * state.remaining_time)
        else {
            break;
        };

        let origin = state.position;

        let mut surface = None;
        let Some(hit) = sweep(
            shape,
            origin,
            rotation,
            direction,
            max_distance,
            config.skin_width,
            query_pipeline,
            filter,
            true,
            |hit| {
                surface = get_surface(hit);
                surface.is_some()
            },
        )
        .map(|hit| SweepHitData {
            // Clamping the distance has its own downsides, but it generally behaves nicer
            distance: hit.distance.max(0.0),
            ..hit
        }) else {
            state.position += direction * max_distance;
            break;
        };

        let surface = surface.unwrap();

        state.remaining_time *= 1.0 - hit.distance / max_distance;
        state.position += direction * hit.distance;

        let impact = MovementHitData {
            origin,
            direction,
            max_distance,
            distance: hit.distance,
            entity: hit.entity,
            point: hit.point,
            surface,
        };

        match on_hit(state, impact) {
            CollisionResponse::Slide => {}
            CollisionResponse::Skip => continue,
            CollisionResponse::Stop => break,
        }

        if surface.is_walkable {
            state.ground = Some(Ground::new(hit.entity, hit.normal));
        }

        state.velocity = collision_state.update(
            surface,
            state.velocity,
            mem::replace(&mut previous_velocity, state.velocity),
            is_grounded,
            |vel| project_velocity(vel, surface),
        );
    }
}

#[derive(Reflect, Debug, Clone, Copy)]
#[reflect(Debug, Clone)]
pub struct MovementHitData {
    pub origin: Vec3,
    pub direction: Dir3,
    pub max_distance: f32,
    pub entity: Entity,
    pub distance: f32,
    pub point: Vec3,
    pub surface: Surface,
}

impl MovementHitData {
    pub fn normal(&self) -> Dir3 {
        self.surface.normal
    }
}

#[derive(Reflect, Debug, PartialEq, Clone, Copy)]
#[reflect(Debug, PartialEq, Clone)]
pub enum CollisionResponse {
    Slide,
    Skip,
    Stop,
}

#[derive(Reflect, Debug, Clone, Copy)]
#[reflect(Debug, Clone)]
pub struct MovementState {
    pub velocity: Vec3,
    pub position: Vec3,
    pub remaining_time: f32,
    pub ground: Option<Ground>,
}

impl MovementState {
    pub fn new(velocity: Vec3, position: Vec3, duration: f32) -> Self {
        Self {
            velocity,
            position,
            remaining_time: duration,
            ground: None,
        }
    }
}

// Maybe this should be a resource rather than a component?
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Default)]
#[require(CollideAndSlideFilter)]
pub struct CollideAndSlideConfig {
    pub max_iterations: u8,
    pub skin_width: f32,
}

impl Default for CollideAndSlideConfig {
    fn default() -> Self {
        Self {
            max_iterations: 4,
            skin_width: 0.02,
        }
    }
}

/// Cache the [`SpatialQueryFilter`] of the character to avoid re-allocating the excluded entities map every time it's used.
#[derive(Component, Reflect, Default, Debug, Clone)]
#[reflect(Component)]
pub struct CollideAndSlideFilter(pub(crate) SpatialQueryFilter);

fn init_filter_mask_on_insert_collision_layers(
    trigger: Trigger<OnInsert, CollisionLayers>,
    mut rigid_bodies: Query<(&mut CollideAndSlideFilter, &CollisionLayers)>,
) {
    let Ok((mut filter, layers)) = rigid_bodies.get_mut(trigger.target()) else {
        return;
    };

    filter.0.mask = layers.filters;
}

fn add_to_filter_on_insert_collider(
    trigger: Trigger<OnInsert, ColliderOf>,
    colliders: Query<&ColliderOf>,
    mut filters: Query<&mut CollideAndSlideFilter>,
) {
    let collider_of = colliders.get(trigger.target()).unwrap();

    let Ok(mut filters) = filters.get_mut(collider_of.body) else {
        return;
    };

    filters.0.excluded_entities.insert(trigger.target());
}

fn remove_from_filter_on_replace_collider(
    trigger: Trigger<OnReplace, ColliderOf>,
    colliders: Query<&ColliderOf>,
    mut filters: Query<&mut CollideAndSlideFilter>,
) {
    let collider_of = colliders.get(trigger.target()).unwrap();

    let Ok(mut filters) = filters.get_mut(collider_of.body) else {
        return;
    };

    filters.0.excluded_entities.remove(trigger.target());
}
