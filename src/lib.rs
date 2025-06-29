use bevy::{
    app::PluginGroupBuilder,
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

pub mod character;
pub mod collide_and_slide;
// pub mod debug;
pub mod grounding;
pub mod movement;
pub mod moving_platform;
pub mod physics_interaction;
pub mod projection;
pub mod stepping;
pub mod sweep;

pub mod prelude {
    pub use crate::{
        CharacterPlugins,
        character::{
            Character, CharacterHit, CharacterStep, GroundEnter, GroundLeave, KinematicVelocity,
            Projectile,
        },
        collide_and_slide::CollideAndSlideConfig,
        grounding::{Ground, Grounding, GroundingConfig},
        movement::{
            CharacterBounce, CharacterDrag, CharacterFriction, CharacterGravity, CharacterMovement,
            MoveInput, MovementPlugin,
        },
        moving_platform::{InheritedVelocity, MovingPlatformPlugin, PhysicsMover},
        stepping::{SteppingBehaviour, SteppingConfig},
    };
}

pub mod type_registration;

pub struct CharacterPlugins {
    pub schedule: Interned<dyn ScheduleLabel>,
}

impl Default for CharacterPlugins {
    fn default() -> Self {
        Self::new(FixedPostUpdate.intern())
    }
}

impl CharacterPlugins {
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl PluginGroup for CharacterPlugins {
    fn build(self) -> PluginGroupBuilder {
        use crate::{
            character::CharacterPlugin, collide_and_slide::CollideAndSlidePlugin,
            movement::MovementPlugin, moving_platform::MovingPlatformPlugin,
            physics_interaction::PhysicsInteractionPlugin,
            type_registration::CharacterTypeRegistrationPlugin,
        };

        PluginGroupBuilder::start::<Self>()
            .add(CharacterTypeRegistrationPlugin)
            .add(CollideAndSlidePlugin)
            .add(CharacterPlugin)
            .add(MovingPlatformPlugin::new(self.schedule))
            .add(PhysicsInteractionPlugin)
            .add(MovementPlugin)
    }
}
