use crate::{
    ground::GroundingConfig,
    sweep::{SweepHitData, sweep},
};
use avian3d::prelude::*;
use bevy::prelude::*;

const STEP_EPSILON: f32 = 1e-4;

#[derive(Debug, Clone, Copy)]
pub(crate) struct StepOutput {
    pub horizontal: f32,
    pub vertical: f32,
    pub hit: SweepHitData,
}

pub(crate) fn step(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    forward_motion: f32,
    up: Dir3,
    skin_width: f32,
    config: &SteppingConfig,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    can_step: impl FnMut(SweepHitData) -> bool,
) -> Option<StepOutput> {
    // Validate input
    if !config.is_valid() || forward_motion <= 0.0 {
        return None;
    }

    let step_up = step_up(
        shape,
        origin,
        rotation,
        up,
        skin_width,
        config.max_vertical,
        filter,
        spatial_query,
    )?;

    step_forward(
        shape,
        origin,
        rotation,
        direction,
        forward_motion,
        up,
        step_up,
        skin_width,
        config,
        filter,
        spatial_query,
        can_step,
    )
}

fn step_up(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    up: Dir3,
    skin_width: f32,
    max_step_up: f32,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
) -> Option<f32> {
    let mut step_up = max_step_up;

    if let Some(hit) = sweep(
        shape,
        origin,
        rotation,
        up,
        max_step_up,
        skin_width,
        spatial_query,
        filter,
        false,
    ) {
        // Hit roof during sweep
        step_up = hit.distance;
    }

    // Head is already touching a roof or wall
    if step_up < STEP_EPSILON {
        return None;
    }

    Some(step_up)
}

fn step_forward(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    forward_motion: f32,
    up: Dir3,
    step_up: f32,
    skin_width: f32,
    config: &SteppingConfig,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    mut can_step: impl FnMut(SweepHitData) -> bool,
) -> Option<StepOutput> {
    let mut min_valid_step: Option<StepOutput> = None;
    let mut min_step_forward = forward_motion;
    let mut step_forward = forward_motion;

    let max_iterations = config.max_iterations + 1;
    let step_size = config.max_horizontal / config.max_iterations.max(1) as f32;

    // Try to find the minimum step forward amount that is still steppable
    for i in 0..max_iterations {
        // Sweep forward
        let mut hit_wall = false;
        if let Some(hit) = sweep(
            shape,
            origin + up * step_up,
            rotation,
            direction,
            step_forward,
            skin_width,
            spatial_query,
            filter,
            false,
        ) {
            step_forward = hit.distance;
            hit_wall = true;
        }

        // Sweep down
        let mut valid_step = None;
        if let Some(hit) = sweep(
            shape,
            origin + up * step_up + direction * step_forward,
            rotation,
            -up,
            step_up + skin_width,
            skin_width,
            spatial_query,
            filter,
            false,
        ) {
            if step_up - hit.distance > STEP_EPSILON && can_step(hit) {
                // We can stand here
                valid_step = Some(StepOutput {
                    horizontal: step_forward,
                    vertical: step_up - hit.distance + skin_width,
                    hit,
                });

                if i == 0 {
                    // info!("initial step is valid");
                    return valid_step;
                }
            }
        }

        match valid_step {
            None => {
                // info!("false -> {}", step_forward);
                min_step_forward = step_forward;
            }
            Some(valid_step) => {
                // info!("true -> {}", step_forward);

                if let Some(last_min_valid_step) = min_valid_step.replace(valid_step) {
                    if last_min_valid_step.horizontal - step_forward < STEP_EPSILON {
                        // info!("threshold reached");
                        break;
                    }
                }
            }
        }

        if hit_wall {
            // info!("hit wall");
            break;
        }

        match min_valid_step {
            // Step a little further
            None => step_forward += step_size,
            // Step to the middle of the furthest invalid step and the closest valid step
            Some(min_valid_step) => {
                step_forward = (min_step_forward + min_valid_step.horizontal) / 2.0;
            }
        }
    }

    min_valid_step
}

/// The step movement of the previous iteration.
#[derive(Component, Reflect, Default, Debug, Clone)]
#[reflect(Component, Debug, Clone)]
pub(crate) struct StepDelta {
    pub vertical: f32,
    pub horizontal: f32,
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
#[require(GroundingConfig, SteppingBehaviour, StepDelta)]
pub struct SteppingConfig {
    pub max_vertical: f32,
    pub max_horizontal: f32,
    pub max_iterations: usize,
}

impl Default for SteppingConfig {
    fn default() -> Self {
        Self {
            max_vertical: 0.25,
            max_horizontal: 0.4,
            max_iterations: 8,
        }
    }
}

impl SteppingConfig {
    pub fn is_valid(&self) -> bool {
        self.max_vertical > 0.0 && self.max_horizontal >= 0.0
    }
}
