use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    grounding::ground_surface_rays,
    sweep::{SweepHitData, sweep},
};

const STEP_EPSILON: f32 = 1e-4;

#[derive(Reflect, Debug, Clone, Copy)]
#[reflect(Debug, Clone)]
pub(crate) struct StepOutput {
    pub horizontal: f32,
    pub vertical: f32,
    pub hit: SweepHitData,
}

pub(crate) fn perform_step(
    config: &SteppingConfig,
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    forward_motion: f32,
    up: Dir3,
    skin_width: f32,
    query_pipeline: &SpatialQueryPipeline,
    query_filter: &SpatialQueryFilter,
    mut filter_hits: impl FnMut(&SweepHitData) -> bool,
    is_walkable: impl FnMut(&SweepHitData) -> bool,
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
        query_pipeline,
        query_filter,
        |hit| filter_hits(hit),
    )?;

    step_forward(
        config,
        shape,
        origin,
        rotation,
        direction,
        forward_motion,
        up,
        step_up,
        skin_width,
        query_pipeline,
        query_filter,
        filter_hits,
        is_walkable,
    )
}

fn step_up(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    up: Dir3,
    skin_width: f32,
    max_step_up: f32,
    spatial_query: &SpatialQueryPipeline,
    query_filter: &SpatialQueryFilter,
    filter_hits: impl FnMut(&SweepHitData) -> bool,
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
        query_filter,
        false,
        filter_hits,
    ) {
        // Hit roof during sweep
        step_up = hit.distance.max(0.0);
    }

    // Head is already touching a roof or wall
    if step_up < STEP_EPSILON {
        return None;
    }

    Some(step_up)
}

fn step_forward(
    config: &SteppingConfig,
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    horizontal_direction: Dir3,
    forward_motion: f32,
    up_direction: Dir3,
    step_up: f32,
    skin_width: f32,
    query_pipeline: &SpatialQueryPipeline,
    query_filter: &SpatialQueryFilter,
    mut filter_hits: impl FnMut(&SweepHitData) -> bool,
    mut validate_step: impl FnMut(&SweepHitData) -> bool,
) -> Option<StepOutput> {
    let mut min_valid_step: Option<StepOutput> = None;
    let mut min_step_forward = forward_motion;
    let mut step_forward = forward_motion;

    let step_size = config.max_horizontal / config.max_substeps.max(1) as f32;

    // Try to find the minimum step forward amount that is still steppable
    for i in 0..config.max_substeps + 1 {
        let step_up_position = origin + up_direction * step_up;

        // Sweep forward
        let mut hit_wall = false;
        if let Some(hit) = sweep(
            shape,
            step_up_position,
            rotation,
            horizontal_direction,
            step_forward,
            skin_width,
            query_pipeline,
            query_filter,
            false,
            |hit| filter_hits(hit),
        ) {
            // Already touching wall
            if hit.distance <= 0.0 {
                break;
            }

            step_forward = hit.distance;
            hit_wall = true;
        }

        let step_forward_position = step_up_position + horizontal_direction * step_forward;

        // Sweep down
        let mut valid_step = None;
        if let Some(mut hit) = sweep(
            shape,
            step_forward_position,
            rotation,
            -up_direction,
            step_up + skin_width,
            skin_width,
            query_pipeline,
            query_filter,
            false,
            |hit| filter_hits(hit),
        ) && hit.distance > 0.0
            && step_up - hit.distance > skin_width
        {
            if let Some(ray_hit) = ground_surface_rays(
                hit.point,
                hit.normal,
                up_direction,
                0.01,
                query_pipeline,
                query_filter,
                |s| filter_hits(s),
            ) {
                hit.normal = ray_hit.normal;
            }

            if validate_step(&hit) {
                // We can stand here
                valid_step = Some(StepOutput {
                    horizontal: step_forward,
                    vertical: step_up - hit.distance,
                    hit,
                });

                if i == 0 {
                    return valid_step;
                }
            }
        }

        match valid_step {
            None => {
                min_step_forward = step_forward;
            }
            Some(valid_step) => {
                if let Some(last_min_valid_step) = min_valid_step.replace(valid_step)
                    && last_min_valid_step.horizontal - step_forward < STEP_EPSILON
                {
                    break;
                }
            }
        }

        if hit_wall {
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

/// Determines when the character should attempt to step up.
#[derive(Reflect, Default, Debug, PartialEq, Eq, Clone, Copy)]
#[reflect(Default, Debug, PartialEq, Clone)]
pub enum SteppingBehaviour {
    Never,
    #[default]
    Grounded,
    Always,
}

/// Configure stepping for a character.
#[derive(Component, Reflect, Debug, PartialEq, Clone, Copy)]
#[reflect(Component, Default, PartialEq, Clone)]
pub struct SteppingConfig {
    pub max_vertical: f32,
    pub max_horizontal: f32,
    pub max_substeps: usize,
    pub behaviour: SteppingBehaviour,
}

impl Default for SteppingConfig {
    fn default() -> Self {
        Self {
            behaviour: Default::default(),
            max_vertical: 0.25,
            max_horizontal: 0.4,
            max_substeps: 8,
        }
    }
}

impl SteppingConfig {
    pub fn is_valid(&self) -> bool {
        self.max_vertical > 0.0 && self.max_horizontal >= 0.0
    }
}
