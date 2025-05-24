use std::mem;

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    CollideAndSlideFilter, CollisionState, ground::Ground, is_walkable, projection::Surface,
};

#[derive(Reflect, Debug, Clone, Copy)]
pub(crate) struct SweepHitData {
    pub distance: f32,
    pub point: Vec3,
    pub normal: Vec3,
    pub entity: Entity,
}

/// Returns the safe hit distance and the hit data from the spatial query.
#[must_use]
pub(crate) fn sweep(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    max_distance: f32,
    skin_width: f32,
    spatial_query: &SpatialQuery,
    filter: &SpatialQueryFilter,
    ignore_origin_penetration: bool,
) -> Option<SweepHitData> {
    let hit = spatial_query.cast_shape(
        shape,
        origin,
        rotation,
        direction,
        &ShapeCastConfig {
            max_distance: max_distance + skin_width, // extend the trace slightly
            target_distance: skin_width, // I'm not sure what this does but I think this is correct ;)
            ignore_origin_penetration,
            ..Default::default()
        },
        filter,
    )?;

    // How far is safe to translate by
    // let distance = hit.distance - skin_width;
    let distance = (hit.distance - skin_width).max(0.0);

    Some(SweepHitData {
        distance,
        point: hit.point1,
        normal: hit.normal1,
        entity: hit.entity,
    })
}

pub(crate) fn step_check(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    motion: f32,
    up: Dir3,
    walkable_angle: f32,
    mut max_forward: f32,
    min_height: f32,
    mut max_height: f32,
    skin_width: f32,
    spatial_query: &SpatialQuery,
    filter: &SpatialQueryFilter,
) -> Option<(Vec3, SweepHitData)> {
    if max_height <= 0.0 {
        return None;
    }

    // check for roof
    if let Some(hit) = sweep(
        shape,
        origin,
        rotation,
        up,
        max_height,
        skin_width,
        spatial_query,
        filter,
        false,
    ) {
        max_height = hit.distance;
    }

    let up_offset = up * max_height;

    // check for wall
    if let Some(hit) = sweep(
        shape,
        origin + up_offset,
        rotation,
        direction,
        max_forward,
        skin_width,
        spatial_query,
        filter,
        false,
    ) {
        max_forward = hit.distance;
    }

    let forward_offset = direction * motion.min(max_forward);

    // check for base step
    if let Some(hit) = sweep(
        shape,
        origin + up_offset + forward_offset,
        rotation,
        -up,
        max_height - min_height,
        skin_width,
        spatial_query,
        filter,
        true,
    ) {
        let is_walkable = is_walkable(hit.normal, walkable_angle, *up);
        if is_walkable {
            let down_offset = -up * hit.distance;
            return Some((up_offset + forward_offset + down_offset, hit));
        }
    }

    // loop check for floor
    let mut forward = 0.0;
    let mut least_forward = 0.0;

    let mut result = None;

    for _ in 0..8 {
        let forward_offset = direction * forward;

        if let Some(hit) = sweep(
            shape,
            origin + up_offset + forward_offset,
            rotation,
            -up,
            max_height - min_height,
            skin_width,
            spatial_query,
            filter,
            true,
        ) {
            let down_offset = -up * hit.distance;

            let is_walkable = is_walkable(hit.normal, walkable_angle - 0.01, *up);

            if is_walkable {
                result = Some((up_offset + forward_offset + down_offset, hit));
                max_forward = forward;
                forward = (forward + least_forward) / 2.0;
                continue;
            }
        }

        least_forward = forward;
        forward = (forward + max_forward) / 2.0;
    }

    result
}

#[derive(Debug, Clone)]
pub(crate) struct MovementState {
    pub velocity: Vec3,
    pub offset: Vec3,
    pub remaining_time: f32,
    pub ground: Option<Ground>,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct MovementImpact {
    pub start: Vec3,
    pub end: Vec3,
    pub direction: Dir3,
    pub remaining_motion: f32,
    pub hit: SweepHitData,
}

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
            skin_width: 0.1,
        }
    }
}

/// If `on_hit` returns `true`, `project_velocity` will be called and the state velocity will be modified, otherwise it will be skipped.
pub(crate) fn collide_and_slide(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    velocity: Vec3,
    current_ground_normal: Option<Dir3>,
    config: CollideAndSlideConfig,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    delta: f32,
    mut project_velocity: impl FnMut(Vec3, Surface) -> Vec3,
    mut on_hit: impl FnMut(&mut MovementState, MovementImpact) -> Option<Surface>,
) -> MovementState {
    let mut state = MovementState {
        velocity,
        offset: Vec3::ZERO,
        remaining_time: delta,
        ground: None,
    };

    let mut previous_velocity = state.velocity;

    let mut collision_state = CollisionState::default();

    for _ in 0..config.max_iterations {
        let Ok((direction, max_distance)) =
            Dir3::new_and_length(state.velocity * state.remaining_time)
        else {
            break;
        };

        let start = origin + state.offset;

        let Some(hit) = sweep(
            shape,
            start,
            rotation,
            direction,
            max_distance,
            config.skin_width,
            spatial_query,
            filter,
            true,
        ) else {
            state.offset += direction * max_distance;
            break;
        };

        state.remaining_time *= 1.0 - hit.distance / max_distance;
        state.offset += direction * hit.distance;

        let impact = MovementImpact {
            start,
            end: origin + state.offset,
            direction,
            remaining_motion: max_distance - hit.distance,
            hit,
        };

        let Some(surface) = on_hit(&mut state, impact) else {
            continue;
        };

        if surface.is_walkable {
            state.ground = Some(Ground::new(hit.entity, hit.normal));
        }

        state.velocity = collision_state.update(
            surface,
            state.velocity,
            mem::replace(&mut previous_velocity, state.velocity),
            current_ground_normal.is_some(),
            |vel| project_velocity(vel, surface),
        );
    }

    state
}
