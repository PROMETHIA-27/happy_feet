use crate::{
    character, collide_and_slide, ground, movement, moving_platform, projection, stepping,
    sweep,
};
use bevy::prelude::*;

pub struct CharacterTypeRegistrationPlugin;

impl Plugin for CharacterTypeRegistrationPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<(
            character::Character,
            character::Projectile,
            character::KinematicVelocity,
            character::CharacterHit,
            character::CharacterStep,
            character::GroundEnter,
            character::GroundLeave,
        )>();
        app.register_type::<(
            collide_and_slide::MovementState,
            collide_and_slide::MovementHitData,
            collide_and_slide::CollisionResponse,
            collide_and_slide::CollideAndSlideConfig,
            collide_and_slide::CollideAndSlideFilter,
        )>();
        app.register_type::<(ground::Grounding, ground::Ground, ground::GroundingConfig)>();
        app.register_type::<(
            movement::MoveInput,
            movement::CharacterMovement,
            movement::CharacterGravity,
            movement::CharacterFriction,
            movement::CharacterDrag,
            movement::FrictionScale,
        )>();
        app.register_type::<(
            moving_platform::PhysicsMover,
            moving_platform::InheritedVelocity,
        )>();
        app.register_type::<(projection::Surface, projection::CollisionState)>();
        app.register_type::<(
            stepping::SteppingBehaviour,
            stepping::SteppingConfig,
            stepping::StepOutput,
        )>();
        app.register_type::<sweep::SweepHitData>();
    }
}
