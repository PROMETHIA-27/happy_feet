use std::{f32::consts::PI, fmt::Debug};

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    character::CharacterSystems,
    collide_and_slide::{CollideAndSlideConfig, CollideAndSlideFilter},
    projection::Surface,
    sweep::{SweepHitData, sweep},
};

#[derive(SystemSet, Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct GroundingSystems;

pub struct GroundingPlugin;

impl Plugin for GroundingPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(
            PhysicsSchedule,
            GroundingSystems
                .after(CharacterSystems)
                .in_set(NarrowPhaseSet::Last),
        );
        app.add_systems(
            PhysicsSchedule,
            (detect_ground, trigger_grounding_events)
                .chain()
                .in_set(GroundingSystems),
        );
    }
}

/// Triggered when the character becomes grounded during a movement update.
///
/// This is only triggered for the last ground the character touched during the update and will not be triggered
/// if the character was already grounded prior to the start of the update.
#[derive(Event, Reflect, Deref)]
pub struct OnGroundEnter(pub Ground);

/// Triggered when the character becomes ungrounded during a movement update.
///
/// This is only triggered if the character is ungrounded at the end of the update.
#[derive(Event, Reflect, Deref)]
pub struct OnGroundLeave(pub Ground);

fn detect_ground(
    // mut gizmos: Gizmos,
    query_pipeline: Res<SpatialQueryPipeline>,
    mut query: Query<(
        &mut Position,
        &Rotation,
        &mut GroundingState,
        &Grounding,
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
        mut grounding_state,
        grounding,
        grounding_config,
        collider,
        config,
        filter,
    ) in &mut query
    {
        if !grounding.is_grounded() && grounding_state.pending.is_none() {
            continue;
        }

        grounding_state.pending = None;

        // Ignore sensors
        let filter_hits = |hit: &SweepHitData| !sensors.contains(hit.entity);

        // Check for ground
        let Some((surface, hit)) = ground_check(
            collider,
            position.0,
            rotation.0,
            grounding_config.up_direction,
            grounding_config.max_distance,
            config.skin_width,
            grounding_config.max_angle,
            &query_pipeline,
            &filter.0,
            filter_hits,
        ) else {
            continue;
        };

        // let color = match surface.is_walkable {
        //     true => LIGHT_GREEN,
        //     false => CRIMSON,
        // };
        //
        // gizmos.line(hit.point, hit.point + hit.normal * 0.2, color);
        // gizmos.line(
        //     hit.point + hit.normal * 0.2,
        //     hit.point + hit.normal * 0.2 + surface.normal * 0.2,
        //     color,
        // );

        let ground = if surface.is_walkable {
            Ground::new(hit.entity, hit.normal)
        } else {
            continue;
        };

        // Snap to the ground
        if grounding_config.snap_to_surface
            && sweep(
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
            position.0 -= grounding_config.up_direction * hit.distance.max(-0.01);
        }

        grounding_state.pending = Some(ground);
    }
}

fn trigger_grounding_events(
    mut query: Query<(Entity, &mut Grounding, &mut GroundingState)>,
    mut commands: Commands,
) {
    for (entity, mut grounding, mut grounding_state) in &mut query {
        match (grounding_state.previous, grounding_state.pending) {
            (Some(ground), None) => {
                commands.entity(entity).trigger(OnGroundLeave(ground));
            }
            (None, Some(ground)) => {
                commands.entity(entity).trigger(OnGroundEnter(ground));
            }
            _ => {}
        }

        grounding_state.previous = grounding_state.pending;
        *grounding = Grounding::new(grounding_state.pending.take());
    }
}

/// Configuration parameters for character grounding behavior.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Default, Debug, Clone)]
#[require(Grounding)]
pub struct GroundingConfig {
    pub up_direction: Dir3,
    /// Max walkable angle
    pub max_angle: f32,
    /// Max distance from the ground
    pub max_distance: f32,
    pub snap_to_surface: bool,
}

impl Default for GroundingConfig {
    fn default() -> Self {
        Self {
            up_direction: Dir3::Y,
            max_angle: PI / 4.0,
            max_distance: 0.2,
            snap_to_surface: true,
        }
    }
}

/// Stores the previous ground state of a character.
#[derive(Component, Reflect, Default, Debug, Clone, Copy)]
#[reflect(Component, Default, Debug, Clone)]
pub struct GroundingState {
    pub previous: Option<Ground>,
    pub pending: Option<Ground>,
}

/// Represents the current ground state of a character.
#[derive(Component, Reflect, Default, Debug, PartialEq, Clone, Copy)]
#[reflect(Component, Default, Debug, PartialEq, Clone)]
#[require(GroundingState)]
pub struct Grounding {
    /// The current ground state, if any.
    pub(crate) inner_ground: Option<Ground>,
    /// Controls whether the character should be forced to detach from the ground, e.g., after jumping.
    should_detach: bool,
}

impl Grounding {
    /// Creates a new [`Grounding`] instance with an optional ground surface.
    pub fn new(surface: Option<Ground>) -> Self {
        Self {
            inner_ground: surface,
            ..Default::default()
        }
    }

    /// Returns whether the character should be forced to detach from its current ground.
    ///
    /// Returns `true` if detachment is requested (e.g., after jumping), `false` otherwise.
    pub fn should_detach(&self) -> bool {
        self.should_detach
    }

    /// Returns whether the character is currently grounded
    ///
    /// A character is considered grounded when it is in contact with a ground surface (`inner_ground` is `Some`)
    /// and is not marked for detachment (`should_detach` is `false`).
    pub fn is_grounded(&self) -> bool {
        !self.should_detach && self.inner_ground.is_some()
    }

    /// Returns the current ground state of the character.
    ///
    /// This method respects the grounding state and will return `None` if:
    /// - There is no ground surface (`inner_ground` is `None`)
    /// - The character is marked for detachment (`should_detach` is `true`)
    pub fn ground(&self) -> Option<Ground> {
        if self.should_detach {
            return None;
        }
        self.inner_ground
    }

    /// Detach from the ground if currently grounded and returns the current ground state.
    ///
    /// This method should be called when the character needs to intentionally leave the ground,
    /// typically when initiating a jump.
    ///
    /// ```
    /// # use bevy::prelude::*;
    /// # use happy_feet::prelude::*;
    /// #
    /// # let mut grounding = Grounding::default();
    /// # let mut velocity = Vec3::ZERO;
    /// # let key = ButtonInput::default();
    /// #
    /// if grounding.is_grounded() && key.just_pressed(KeyCode::Space) {
    ///     velocity.y = 10.0;
    ///     grounding.detach();
    /// }
    /// ```
    pub fn detach(&mut self) -> Option<Ground> {
        if !self.is_grounded() {
            return None;
        }

        self.should_detach = true;
        self.inner_ground
    }

    /// Returns the normal vector of the current ground surface if grounded.
    ///
    /// This method respects the grounding state and will return `None` if:
    /// - There is no ground surface (`inner_ground` is `None`)
    /// - The character is marked for detachment (`should_detach` is `true`)
    pub fn normal(&self) -> Option<Dir3> {
        self.ground().map(|ground| ground.normal)
    }

    /// Returns the [`Entity`] of the current ground surface if grounded.
    ///
    /// This method respects the grounding state and will return `None` if:
    /// - There is no ground surface (`inner_ground` is `None`)
    /// - The character is marked for detachment (`should_detach` is `true`)
    pub fn entity(&self) -> Option<Entity> {
        self.ground().map(|ground| ground.entity)
    }

    /// Returns the internal ground state without checking detachment status.
    pub fn inner_ground(&self) -> Option<Ground> {
        self.inner_ground
    }
}

impl From<Option<Ground>> for Grounding {
    fn from(value: Option<Ground>) -> Self {
        Self {
            inner_ground: value,
            ..Default::default()
        }
    }
}

impl From<Ground> for Grounding {
    fn from(value: Ground) -> Self {
        Self::from(Some(value))
    }
}

/// Represents a surface that a character can stand on.
#[derive(Reflect, Debug, PartialEq, Clone, Copy)]
#[reflect(Debug, PartialEq, Clone)]
pub struct Ground {
    /// The surface normal vector, pointing outward from the surface.
    pub normal: Dir3,
    pub entity: Entity,
}

impl Ground {
    pub fn new<D>(entity: Entity, normal: D) -> Self
    where
        D: TryInto<Dir3>,
        <D as TryInto<Dir3>>::Error: Debug,
    {
        Self {
            entity,
            normal: normal.try_into().unwrap(),
        }
    }

    /// Construct a new [`Ground`] if the `normal` is walkable with the given `walkable_angle` and `up` direction.
    pub fn new_if_walkable(
        entity: Entity,
        normal: impl TryInto<Dir3>,
        up: Dir3,
        walkable_angle: f32,
    ) -> Option<Self> {
        let normal = normal.try_into().ok()?;

        if !is_walkable(*normal, walkable_angle, *up) {
            return None;
        }

        Some(Self { entity, normal })
    }
}

/// Sweep in the opposite direction of `up` and return the [`Ground`] if it's walkable.
pub(crate) fn ground_check(
    collider: &Collider,
    translation: Vec3,
    rotation: Quat,
    up_direction: Dir3,
    max_distance: f32,
    skin_width: f32,
    walkable_angle: f32,
    query_pipeline: &SpatialQueryPipeline,
    query_filter: &SpatialQueryFilter,
    mut filter_hits: impl FnMut(&SweepHitData) -> bool,
) -> Option<(Surface, SweepHitData)> {
    let hit = sweep(
        collider,
        translation,
        rotation,
        -up_direction,
        max_distance,
        skin_width,
        query_pipeline,
        query_filter,
        true,
        |hit| filter_hits(hit),
    )?;

    let mut normal = hit.normal;

    if let Some(ray_hit) = ground_surface_rays(
        hit.point,
        hit.normal,
        up_direction,
        0.01,
        query_pipeline,
        query_filter,
        |sweep| filter_hits(sweep),
    ) && ray_hit.normal.dot(*up_direction) > hit.normal.dot(*up_direction)
    {
        normal = ray_hit.normal;
    }

    Some((Surface::new(normal, walkable_angle, up_direction), hit))
}

/// Attempt to retrieve better surface normal using ray casts.
pub(crate) fn ground_surface_rays(
    point: Vec3,
    normal: Vec3,
    up_direction: Dir3,
    epsilon: f32,
    query_pipeline: &SpatialQueryPipeline,
    query_filter: &SpatialQueryFilter,
    mut filter_hits: impl FnMut(&SweepHitData) -> bool,
) -> Option<SweepHitData> {
    let mut get_sweep_hit = |ray_hit: &RayHitData| {
        let sweep = SweepHitData {
            point: point - up_direction * ray_hit.distance,
            normal: ray_hit.normal,
            distance: ray_hit.distance,
            entity: ray_hit.entity,
        };
        filter_hits(&sweep).then_some(sweep)
    };

    let mut hit = None;

    // Early exit if up and normal are parallel
    let right = up_direction.cross(normal).try_normalize()?;
    let forward = up_direction.cross(right).try_normalize()?;

    // TODO: step 0 could probably be removed
    for i in 0..3 {
        let origin = match i {
            0 => point + up_direction * epsilon / 2.0,
            1 => point + (up_direction * 0.5 + forward) * epsilon,
            2 => point + (up_direction * 0.5 - forward) * epsilon,
            _ => unreachable!(),
        };

        let mut new_hit = None;
        query_pipeline.ray_hits_callback(
            origin,
            -up_direction,
            epsilon,
            false,
            query_filter,
            |ray_hit| {
                if let Some(sweep) = get_sweep_hit(&ray_hit) {
                    new_hit = Some(sweep);
                }
                new_hit.is_none()
            },
        );

        match (hit, new_hit) {
            (None, Some(new_hit)) => hit = Some(new_hit),
            (Some(old_hit), Some(new_hit)) => {
                if new_hit.normal.dot(*up_direction) > old_hit.normal.dot(*up_direction) {
                    hit = Some(new_hit)
                }
            }
            _ => {}
        }
    }

    hit
}

// TODO: remove this and use Surface::new(..).is_walkable instead ???
pub(crate) fn is_walkable(normal: Vec3, walkable_angle: f32, up: Vec3) -> bool {
    normal.angle_between(up) <= walkable_angle
}
