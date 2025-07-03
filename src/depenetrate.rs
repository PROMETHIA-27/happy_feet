use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    character::{CharacterSystems, KinematicVelocity},
    grounding::{Ground, Grounding, GroundingConfig, OnGroundEnter},
    projection::{Surface, project_velocity},
};

#[derive(SystemSet, Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct DepenetrateSystems;

pub struct DepenetratePlugin;

impl Plugin for DepenetratePlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(
            PhysicsSchedule,
            (
                DepenetrateSystems.in_set(NarrowPhaseSet::Last),
                CharacterSystems,
            )
                .chain(),
        );

        app.add_systems(PhysicsSchedule, depenetrate.in_set(DepenetrateSystems));
    }
}

pub(crate) fn depenetrate(
    mut commands: Commands,
    mut overlaps: Local<Vec<(Dir3, f32)>>,
    collisions: Collisions,
    mut query: Query<(
        Entity,
        &mut Position,
        &mut KinematicVelocity,
        Option<(&mut Grounding, &GroundingConfig)>,
    )>,
    colliders: Query<&ColliderOf, Without<Sensor>>,
) {
    for contacts in collisions.iter() {
        overlaps.clear();

        // Get the rigid body entities of the colliders (colliders could be children)
        let Ok([&ColliderOf { body: rb1 }, &ColliderOf { body: rb2 }]) =
            colliders.get_many([contacts.collider1, contacts.collider2])
        else {
            continue;
        };

        let other: Entity;

        let (entity, mut position, mut velocity, mut grounding) =
            if let Ok(character) = query.get_mut(rb1) {
                other = rb2;
                character
            } else if let Ok(character) = query.get_mut(rb2) {
                other = rb1;
                character
            } else {
                continue;
            };

        // TODO: crease / corner handling (?)

        for manifold in &contacts.manifolds {
            let hit_normal = match entity == rb1 {
                true => -manifold.normal,
                false => manifold.normal,
            };

            let (surface, obstruction_normal, ground_normal) = match grounding.as_ref() {
                Some((grounding, grounding_config)) => {
                    let surface = Surface::new(
                        hit_normal,
                        grounding_config.max_angle,
                        grounding_config.up_direction,
                    );
                    let ground_normal = grounding.normal();
                    let obstruction_normal = surface
                        .obstruction_normal(ground_normal, grounding_config.up_direction)
                        .unwrap();
                    (surface, obstruction_normal, ground_normal)
                }
                None => {
                    let surface = Surface {
                        normal: Dir3::new(hit_normal).unwrap(),
                        is_walkable: false,
                    };
                    (surface, surface.normal, None)
                }
            };

            for contact in &manifold.points {
                let depth = contact.penetration * hit_normal.dot(*obstruction_normal);

                match overlaps.binary_search_by(|(_, d)| depth.total_cmp(d)) {
                    Ok(index) => {
                        if overlaps[index].0.dot(*obstruction_normal) > 1.0 - 1e-4 {
                            overlaps.push((obstruction_normal, depth));
                            overlaps.swap_remove(index);
                        } else {
                            overlaps.insert(index, (obstruction_normal, depth));
                        }
                    }
                    Err(index) => {
                        overlaps.insert(index, (obstruction_normal, depth));
                    }
                }
            }

            if velocity.0.dot(hit_normal) > 0.0 {
                continue;
            }

            match grounding {
                Some((_, grounding_config)) => {
                    velocity.0 = project_velocity(
                        velocity.0,
                        *obstruction_normal,
                        surface.is_walkable,
                        ground_normal,
                        grounding_config.up_direction,
                    );
                }
                None => {
                    velocity.0 = velocity.0.reject_from(*obstruction_normal);
                }
            }

            if surface.is_walkable
                && let Some((grounding, _)) = grounding.as_mut()
            {
                let ground = Ground::new(other, hit_normal);

                if !grounding.is_grounded() {
                    commands.entity(entity).trigger(OnGroundEnter(ground));
                }

                **grounding = ground.into();
            }
        }

        // TODO: This is probably not necessary
        for i in 0..overlaps.len() {
            let (direction, depth) = overlaps[i];

            if depth <= 0.0 {
                continue;
            }

            position.0 += direction * depth;

            for j in i..overlaps.len() {
                let (next_direction, ref mut next_depth) = overlaps[j];

                let fixed = f32::max(0.0, direction.dot(*next_direction) * depth);
                *next_depth -= fixed;
            }
        }
    }
}
