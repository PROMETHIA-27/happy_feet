use std::fmt::Debug;

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::sweep;

/// The ground state of a character.
#[derive(Reflect, Default, Debug, PartialEq, Clone, Copy)]
pub struct CharacterGrounding {
    pub(crate) inner_surface: Option<GroundSurface>,
    /// If the character should be forced to unground, e.g., after jumping.
    should_detach: bool,
}

impl CharacterGrounding {
    pub fn new(surface: Option<GroundSurface>) -> Self {
        Self {
            inner_surface: surface,
            ..Default::default()
        }
    }

    pub fn from_surface(surface: GroundSurface) -> Self {
        Self {
            inner_surface: Some(surface),
            ..Default::default()
        }
    }

    pub fn is_stable(&self) -> bool {
        !self.should_detach && self.inner_surface.is_some()
    }

    pub fn stable_surface(&self) -> Option<GroundSurface> {
        if self.should_detach {
            None
        } else {
            self.inner_surface
        }
    }

    pub fn clear_surface(&mut self) {
        *self = Self::default();
    }

    pub fn set_surface(&mut self, ground: GroundSurface) {
        *self = Self::from_surface(ground);
    }

    pub fn detach_from_surface(&mut self) -> Option<GroundSurface> {
        if !self.is_stable() {
            return None;
        }

        self.should_detach = true;
        self.inner_surface
    }

    pub fn normal(&self) -> Option<Dir3> {
        self.stable_surface().map(|ground| ground.normal)
    }
}

impl From<GroundSurface> for CharacterGrounding {
    fn from(value: GroundSurface) -> Self {
        Self::from_surface(value)
    }
}

/// Represents a surface that a character can stand on.
#[derive(Reflect, Debug, PartialEq, Clone, Copy)]
pub struct GroundSurface {
    /// The surface normal vector, pointing outward from the surface.
    pub normal: Dir3,
    pub entity: Entity,
}

impl GroundSurface {
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
) -> Option<(f32, GroundSurface)> {
    let (distance, hit) = sweep(
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

    let ground = GroundSurface::new_if_walkable(hit.entity, hit.normal1, up, walkable_angle)?;

    Some((distance, ground))
}

pub(crate) fn is_walkable(normal: Vec3, walkable_angle: f32, up: Vec3) -> bool {
    normal.angle_between(up) <= walkable_angle
}
