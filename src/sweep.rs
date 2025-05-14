use avian3d::prelude::*;
use bevy::prelude::*;

use crate::project::SlidePlanes;

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
    let safe_distance = hit.distance - skin_width;

    Some((safe_distance, hit))
}

#[derive(Debug, Clone)]
pub(crate) struct MoveAndSlideState<const SIZE: usize> {
    pub velocities: [Vec3; SIZE],
    pub offset: Vec3,
    pub remaining_time: f32,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct MoveImpact {
    pub origin: Vec3,
    pub direction: Dir3,
    pub incoming: f32,
    pub remaining: f32,
    pub hit: ShapeHitData,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct MoveAndSlideResult {
    pub offset: Vec3,
    pub velocity: Vec3,
}

pub(crate) fn move_and_slide<const SIZE: usize>(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    velocities: [Vec3; SIZE],
    max_slide_count: usize,
    skin_width: f32,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    delta: f32,
    mut on_hit: impl FnMut(&mut MoveAndSlideState<SIZE>, MoveImpact) -> bool,
) -> MoveAndSlideState<SIZE> {
    let mut state = MoveAndSlideState {
        velocities,
        offset: Vec3::ZERO,
        remaining_time: delta,
    };

    let Ok(original_direction) = Dir3::new(state.velocities.iter().sum()) else {
        return state;
    };

    let mut planes = SlidePlanes::default();

    for _ in 0..max_slide_count {
        let Ok((direction, max_distance)) =
            Dir3::new_and_length(state.velocities.iter().sum::<Vec3>() * state.remaining_time)
        else {
            break;
        };

        let origin = origin + state.offset;

        let Some((distance, hit)) = sweep(
            shape,
            origin,
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

        assert!(state.remaining_time >= 0.0);

        state.offset += direction * distance;

        if !on_hit(
            &mut state,
            MoveImpact {
                origin,
                direction,
                incoming: distance,
                remaining: max_distance - distance,
                hit,
            },
        ) {
            continue;
        }

        if planes.push(hit.normal1) {
            for velocity in &mut state.velocities {
                *velocity = planes.project_velocity(*velocity, original_direction);

                if velocity.dot(*original_direction) <= 0.0 {
                    *velocity = Vec3::ZERO;
                }
            }
        }
    }

    state
}
