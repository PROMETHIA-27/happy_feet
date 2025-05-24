use std::{f32::consts::PI, fmt::Debug};

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::sweep::{SweepHitData, sweep};

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Default)]
#[require(Grounding)]
pub struct GroundingConfig {
    /// Mask for walkable ground, not currently used
    pub layer_mask: Option<LayerMask>,
    /// Max walkable angle
    pub max_angle: f32,
    /// Max distance from the ground
    pub max_distance: f32,
    pub snap_to_surface: bool,
    /// The up direction.
    pub up: Dir3,
}

impl Default for GroundingConfig {
    fn default() -> Self {
        Self {
            layer_mask: None,
            max_angle: PI / 4.0,
            max_distance: 0.2,
            snap_to_surface: true,
            up: Dir3::Y,
        }
    }
}

/// The ground state of a character.
#[derive(Component, Reflect, Default, Debug, PartialEq, Clone, Copy)]
#[reflect(Component, Default)]
pub struct Grounding {
    pub(crate) inner_ground: Option<Ground>,
    /// If the character should be forced to detach from the ground, e.g., after jumping.
    should_detach: bool,
}

impl Grounding {
    pub fn new(surface: Option<Ground>) -> Self {
        Self {
            inner_ground: surface,
            ..Default::default()
        }
    }

    pub fn is_grounded(&self) -> bool {
        !self.should_detach && self.inner_ground.is_some()
    }

    pub fn ground(&self) -> Option<Ground> {
        if self.should_detach {
            return None;
        }
        self.inner_ground
    }

    pub fn clear_ground(&mut self) {
        *self = Self::default();
    }

    pub fn set_ground(&mut self, ground: Ground) {
        *self = Self::from(ground);
    }

    pub fn detach(&mut self) -> Option<Ground> {
        if !self.is_grounded() {
            return None;
        }

        self.should_detach = true;
        self.inner_ground
    }

    pub fn normal(&self) -> Option<Dir3> {
        self.ground().map(|ground| ground.normal)
    }

    pub fn entity(&self) -> Option<Entity> {
        self.ground().map(|ground| ground.entity)
    }

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
    up: Dir3,
    max_distance: f32,
    skin_width: f32,
    walkable_angle: f32,
    spatial_query: &SpatialQuery,
    filter: &SpatialQueryFilter,
) -> Option<(Ground, SweepHitData)> {
    let hit = sweep(
        collider,
        translation,
        rotation,
        -up,
        max_distance,
        skin_width,
        spatial_query,
        filter,
        true,
    )?;

    let ground = Ground::new_if_walkable(hit.entity, hit.normal, up, walkable_angle)?;

    Some((ground, hit))
}

pub(crate) fn is_walkable(normal: Vec3, walkable_angle: f32, up: Vec3) -> bool {
    normal.angle_between(up) <= walkable_angle
}
