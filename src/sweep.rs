use std::{f32::consts::PI, mem, ops::Range};

use avian3d::prelude::*;
use bevy::{
    color::palettes::css::{BLACK, CRIMSON, GREEN, LIGHT_GREEN, RED, WHITE},
    prelude::*,
};

use crate::{CharacterGizmos, CollisionState, SurfacePlane, is_walkable};

/// Result of the move_and_slide function.

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
) -> Option<(f32, ShapeHitData)> {
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

    Some((distance, hit))
}

pub(crate) fn step_check(
    gizmos: &mut Gizmos<CharacterGizmos>,
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
) -> Option<(Vec3, ShapeHitData)> {
    if max_height <= 0.0 {
        return None;
    }

    // check for roof
    if let Some((distance, _)) = sweep(
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
        max_height = distance;
    }

    let up_offset = up * max_height;

    gizmos.line(origin, origin + up_offset, BLACK);

    // check for wall
    if let Some((distance, _)) = sweep(
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
        max_forward = distance;
    }

    gizmos.line(
        origin + up_offset,
        origin + up_offset + direction * max_forward,
        BLACK,
    );

    let forward_offset = direction * motion.min(max_forward);

    // check for base step
    if let Some((distance, hit)) = sweep(
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
        let is_walkable = is_walkable(hit.normal1, *up, walkable_angle - 1e-4);
        if is_walkable {
            let down_offset = -up * distance;
            return Some((up_offset + forward_offset + down_offset, hit));
        }
    }

    // loop check for floor
    let mut forward = 0.0;
    let mut least_forward = 0.0;

    let mut result = None;

    for i in 0..8 {
        let forward_offset = direction * forward;

        if let Some((distance, hit)) = sweep(
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
            let down_offset = -up * distance;

            let is_walkable = is_walkable(hit.normal1, *up, walkable_angle - 1e-4);

            gizmos.line(
                origin + up_offset + forward_offset,
                origin + up_offset + forward_offset + down_offset,
                match is_walkable {
                    true => match i == 7 {
                        true => LIGHT_GREEN,
                        false => GREEN,
                    },
                    false => CRIMSON,
                },
            );

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

pub(crate) fn ledge_check(
    gizmos: &mut Gizmos<CharacterGizmos>,
    up: Dir3,
    walkable_angle: f32,
    hit_point: Vec3,
    hit_normal: Vec3,
    spatial_query: &SpatialQuery,
    filter: &SpatialQueryFilter,
) -> Option<RayHitData> {
    const VERTICAL_OFFSET: f32 = 0.02;
    const HORIZONTAL_OFFSET: f32 = 0.02;
    const LEDGE_CHECK_DISTANCE: f32 = 0.05;

    let horizontal_hit_direction = hit_normal.reject_from(*up).normalize_or_zero();

    let mut ledge_hit = None;

    let inner_origin =
        hit_point + up * VERTICAL_OFFSET + horizontal_hit_direction * HORIZONTAL_OFFSET;
    let outer_origin =
        hit_point + up * VERTICAL_OFFSET - horizontal_hit_direction * HORIZONTAL_OFFSET;
    let max_distance = VERTICAL_OFFSET + LEDGE_CHECK_DISTANCE;

    gizmos.line(inner_origin, outer_origin, WHITE);

    if let Some(inner_hit) = spatial_query.cast_ray(inner_origin, -up, max_distance, false, filter)
    {
        if is_walkable(inner_hit.normal, *up, walkable_angle) {
            ledge_hit = Some(inner_hit);
            gizmos.line(
                inner_origin - up * inner_hit.distance,
                inner_origin - up * (inner_hit.distance - 0.5),
                RED,
            );
        }
    }

    if let Some(outer_hit) = spatial_query.cast_ray(outer_origin, -up, max_distance, false, filter)
    {
        let inner_walkable = is_walkable(outer_hit.normal, *up, walkable_angle);
        if inner_walkable {
            gizmos.line(
                outer_origin - up * outer_hit.distance,
                outer_origin - up * (outer_hit.distance - 0.5),
                RED,
            );
        }

        if inner_walkable && ledge_hit.take().is_none() {
            ledge_hit = Some(outer_hit);
        }
    }

    ledge_hit
}

#[derive(Debug, Clone)]
pub(crate) struct MovementState {
    pub velocity: Vec3,
    pub offset: Vec3,
    pub remaining_time: f32,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct CollideImpact {
    pub start: Vec3,
    pub end: Vec3,
    pub direction: Dir3,
    pub incoming_motion: f32,
    pub remaining_motion: f32,
    pub plane: SurfacePlane,
    pub hit: ShapeHitData,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct MoveAndSlideResult {
    pub offset: Vec3,
    pub velocity: Vec3,
}

pub(crate) fn collide_and_slide(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    velocity: Vec3,
    ground_normal: Option<Vec3>,
    walkable_angle: f32,
    up: Dir3,
    max_slide_count: usize,
    skin_width: f32,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    delta: f32,
    mut project_velocity: impl FnMut(Vec3, SurfacePlane) -> Vec3,
    mut on_hit: impl FnMut(&mut MovementState, CollideImpact) -> bool,
) -> MovementState {
    let mut state = MovementState {
        velocity,
        offset: Vec3::ZERO,
        remaining_time: delta,
    };

    let mut previous_velocity = state.velocity;

    let mut collision_state = CollisionState::default();

    for _ in 0..max_slide_count {
        let Ok((direction, max_distance)) =
            Dir3::new_and_length(state.velocity * state.remaining_time)
        else {
            break;
        };

        let start = origin + state.offset;

        let Some((distance, hit)) = sweep(
            shape,
            start,
            rotation,
            direction,
            max_distance,
            skin_width,
            spatial_query,
            filter,
            true,
        ) else {
            state.offset += direction * max_distance;

            break;
        };

        state.remaining_time *= 1.0 - distance / max_distance;

        state.offset += direction * distance;

        let end = origin + state.offset;

        let plane = SurfacePlane::new(hit.normal1, ground_normal, walkable_angle, up);

        if !on_hit(
            &mut state,
            CollideImpact {
                start,
                end,
                direction,
                incoming_motion: distance,
                remaining_motion: max_distance - distance,
                plane,
                hit,
            },
        ) {
            continue;
        }

        state.velocity = collision_state.slide_with(
            plane,
            state.velocity,
            mem::replace(&mut previous_velocity, state.velocity),
            ground_normal,
            up,
            |vel| project_velocity(vel, plane),
        );
    }

    state
}
