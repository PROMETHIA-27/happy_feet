use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{character_sweep, sweep};

/// Represents the ground a character is currently standing on.
#[derive(Reflect, Debug, PartialEq, Clone, Copy)]
pub struct Ground {
    pub entity: Entity,
    pub normal: Dir3,
}

impl Ground {
    /// Construct a new [`Ground`] if the `normal` is walkable with the given `walkable_angle` and `up` direction.
    pub fn new_if_walkable(
        entity: Entity,
        normal: impl TryInto<Dir3>,
        up: Dir3,
        walkable_angle: f32,
    ) -> Option<Self> {
        let normal = normal.try_into().ok()?;

        if !is_walkable(*normal, *up, walkable_angle) {
            return None;
        }

        Some(Self { entity, normal })
    }

    /// Returns `true` if the [`Ground`] is walkable with the given `walkable_angle` and `up` direction.
    pub fn is_walkable(&self, up: Dir3, walkable_angle: f32) -> bool {
        is_walkable(*self.normal, *up, walkable_angle)
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
) -> Option<(f32, Ground)> {
    let (distance, hit) = character_sweep(
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

    let ground = Ground::new_if_walkable(hit.entity, hit.normal1, up, walkable_angle)?;

    Some((distance, ground))
}

pub(crate) fn is_walkable(normal: Vec3, up: Vec3, walkable_angle: f32) -> bool {
    normal.angle_between(up) <= walkable_angle
}
