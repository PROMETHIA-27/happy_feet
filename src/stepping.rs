use crate::{
    ground::GroundingConfig,
    sweep::{SweepHitData, sweep},
};
use avian3d::prelude::*;
use bevy::prelude::*;

pub(crate) struct StepOutput {
    pub step_forward: f32,
    pub step_up: f32,
    pub hit: SweepHitData,
}

pub(crate) fn step_up(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    mut forward_motion: f32,
    up: Dir3,
    skin_width: f32,
    config: &SteppingConfig,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    mut can_step: impl FnMut(SweepHitData) -> bool,
) -> Option<StepOutput> {
    const EPSILON: f32 = 1e-4;

    if config.max_iterations == 0 || config.max_step_forward <= 0.0 || config.max_step_up <= 0.0 {
        return None;
    }

    // Basically no motion, don't even try to step
    if forward_motion < EPSILON {
        return None;
    }

    let mut position = origin;

    // Step up
    let mut step_up = config.max_step_up;
    if let Some(hit) = sweep(
        shape,
        position,
        rotation,
        up,
        config.max_step_up,
        skin_width,
        spatial_query,
        filter,
        false,
    ) {
        step_up = hit.distance;
    }

    // Head is already touching a roof or wall, can't step here
    if step_up < EPSILON {
        return None;
    }

    position += up * step_up;

    let mut total_step_forward = 0.0;

    let step_size = config.max_step_forward / config.max_iterations as f32;

    for _ in 0..config.max_iterations {
        // Step forward
        let mut current_step_forward = forward_motion;
        if let Some(hit) = sweep(
            shape,
            position,
            rotation,
            direction,
            forward_motion,
            skin_width,
            spatial_query,
            filter,
            false,
        ) {
            current_step_forward = hit.distance;
        }

        // We're stuck :(
        if current_step_forward < EPSILON {
            break;
        }

        position += direction * current_step_forward;

        total_step_forward += current_step_forward;

        // Step down
        if let Some(hit) = sweep(
            shape,
            position,
            rotation,
            -up,
            config.max_step_up,
            skin_width,
            spatial_query,
            filter,
            true,
        ) {
            // Step height is too low
            if step_up - hit.distance < EPSILON {
                break;
            }

            // We can step here!
            if can_step(hit) {
                step_up -= hit.distance;
                return Some(StepOutput {
                    step_forward: total_step_forward,
                    step_up,
                    hit,
                });
            }
        }

        // Stepped too far, give up
        if total_step_forward > config.max_step_forward {
            break;
        }

        // Keep trying!
        forward_motion = step_size;
    }

    None
}

/// Determines when the character should attempt to step up.
#[derive(Component, Reflect, Default, Debug, PartialEq, Eq, Clone, Copy)]
#[reflect(Component, Default)]
pub enum SteppingBehaviour {
    Never,
    #[default]
    Grounded,
    Always,
}

/// Configure stepping for a character.
#[derive(Component, Reflect, Debug, PartialEq, Clone, Copy)]
#[reflect(Component, Default)]
#[require(GroundingConfig, SteppingBehaviour)]
pub struct SteppingConfig {
    pub max_step_up: f32,
    pub max_step_forward: f32,
    pub max_iterations: usize,
}

impl Default for SteppingConfig {
    fn default() -> Self {
        Self {
            max_step_up: 0.25,
            max_step_forward: 0.25,
            max_iterations: 6,
        }
    }
}
