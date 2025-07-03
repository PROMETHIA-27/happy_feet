//! - WASD for movement
//! - Space bar for jumping

use avian3d::prelude::*;
use bevy::{
    color::palettes::css::*,
    image::{ImageAddressMode, ImageSamplerDescriptor},
    input::InputSystem,
    math::Affine2,
    prelude::*,
};
use happy_feet::debug::{DebugGrounding, DebugInput};
use happy_feet::movement::acceleration;
use happy_feet::prelude::*;
use std::collections::HashMap;
use std::f32::consts::PI;

#[derive(Component, Default, Debug, PartialEq, Eq, Hash, Clone, Copy)]
enum PenguinMovementMode {
    Walking,
    #[default]
    Falling,
    Sliding,
}

#[derive(Component, Debug, Clone)]
#[require(CharacterMovement, PenguinMovementMode, GroundFriction)]
struct PenguinMovement {
    walking_acceleration: f32,
    falling_acceleration: f32,
    sliding_acceleration: f32,
    walking_friction: f32,
    sliding_friction: f32,
    jump_impulse: f32,
}

fn update_movement(
    mut query: Query<
        (
            &PenguinMovementMode,
            &PenguinMovement,
            &mut CharacterMovement,
            &mut GroundFriction,
        ),
        Changed<PenguinMovementMode>,
    >,
) {
    for (movement_mode, penguin, mut movement, mut friction) in &mut query {
        movement.acceleration = match movement_mode {
            PenguinMovementMode::Walking => penguin.walking_acceleration,
            PenguinMovementMode::Falling => penguin.falling_acceleration,
            PenguinMovementMode::Sliding => penguin.sliding_acceleration,
        };

        friction.0 = match movement_mode {
            PenguinMovementMode::Walking | PenguinMovementMode::Falling => penguin.walking_friction,
            PenguinMovementMode::Sliding => penguin.sliding_friction,
        };
    }
}

fn main() -> AppExit {
    App::new()
        .add_plugins((
            // This is used for tiling the ground texture
            DefaultPlugins.set(ImagePlugin {
                default_sampler: ImageSamplerDescriptor {
                    address_mode_u: ImageAddressMode::Repeat,
                    address_mode_v: ImageAddressMode::Repeat,
                    ..Default::default()
                },
            }),
            // Make sure the physics plugins and character plugins are configured to run in the same schedule (default is FixedPostUpdate for both)
            PhysicsPlugins::default(),
            CharacterPlugins::default(),
            // This is used for debugging and can be removed
            PhysicsDebugPlugin::default(),
        ))
        .add_systems(Startup, (setup_character, setup_level))
        .add_systems(PreUpdate, character_input.after(InputSystem))
        .add_systems(FixedUpdate, update_movement)
        .add_observer(init_ground_movement)
        .add_observer(init_air_movement)
        .run()
}

fn setup_character(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let shape = Capsule3d::new(0.3, 1.0);
    commands.spawn((
        Character,
        DebugGrounding,
        DebugInput,
        PenguinMovement {
            walking_acceleration: 50.0,
            falling_acceleration: 20.0,
            sliding_acceleration: 0.0,
            walking_friction: 40.0,
            sliding_friction: 10.0,
            jump_impulse: 4.0,
        },
        CharacterMovement {
            target_speed: 5.0,
            ..Default::default()
        },
        SteppingConfig {
            behaviour: SteppingBehaviour::Always,
            ..Default::default()
        },
        // Enable gravity
        CharacterGravity::new(Vec3::Y * -20.0),
        // Transform::from_xyz(0.0, 3.0, 0.0),
        Collider::from(shape),
        Mesh3d(meshes.add(shape)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: MIDNIGHT_BLUE.into(),
            perceptual_roughness: 0.8,
            ..Default::default()
        })),
    ));
}

fn character_input(
    key: Res<ButtonInput<KeyCode>>,
    mut query: Query<(
        &mut MoveInput,
        &mut KinematicVelocity,
        &mut Grounding,
        &mut Transform,
        &PenguinMovement,
        &mut PenguinMovementMode,
    )>,
    time: Res<Time>,
) {
    let x = match (key.pressed(KeyCode::KeyA), key.pressed(KeyCode::KeyD)) {
        (true, false) => -1.0,
        (false, true) => 1.0,
        _ => 0.0,
    };
    let z = match (key.pressed(KeyCode::KeyW), key.pressed(KeyCode::KeyS)) {
        (true, false) => -1.0,
        (false, true) => 1.0,
        _ => 0.0,
    };

    let move_direction = Vec3::new(x, 0.0, z).normalize_or_zero();

    for (mut move_input, mut velocity, mut grounding, mut transform, penguin, mut movement_mode) in
        &mut query
    {
        move_input.value = move_direction;

        if let Ok(direction) = Dir3::new(velocity.0) {
            let target_rotation = match *movement_mode {
                PenguinMovementMode::Walking | PenguinMovementMode::Falling => {
                    Transform::IDENTITY
                        .aligned_by(Dir3::Y, Dir3::Y, Dir3::NEG_Z, direction)
                        .rotation
                }
                PenguinMovementMode::Sliding => {
                    Transform::IDENTITY
                        .aligned_by(Dir3::Z, Dir3::Y, Dir3::Y, direction)
                        .rotation
                }
            };

            transform
                .rotation
                .smooth_nudge(&target_rotation, 8.0, time.delta_secs());
        }

        if grounding.is_grounded() && key.just_pressed(KeyCode::Space) {
            velocity.y = penguin.jump_impulse;
            grounding.detach(); // Detach from the ground to avoid snapping back to it during movement update

            if let PenguinMovementMode::Sliding = *movement_mode {
                transform.rotation *= Quat::from_axis_angle(Vec3::X, PI / 2.0);
                *movement_mode = PenguinMovementMode::Falling;
            }
        }

        // TODO: hold to slide
        if key.just_pressed(KeyCode::ShiftLeft) {
            if *movement_mode == PenguinMovementMode::Walking {
                grounding.detach();
            }

            if *movement_mode != PenguinMovementMode::Sliding {
                // velocity.y = penguin.jump_impulse / 2.0;

                let direction = Dir3::new(move_direction).unwrap_or(transform.forward());

                let target_rotation = Transform::IDENTITY
                    .aligned_by(Dir3::Y, Dir3::Y, Dir3::NEG_Z, direction)
                    .rotation;

                transform.rotation = target_rotation;

                match velocity.dot(*direction) {
                    d if d < 0.0 => {
                        velocity.0 = *direction * penguin.jump_impulse;
                    }

                    d => {
                        velocity.0 = *direction * (penguin.jump_impulse + d / 2.0);
                    }
                }

                if velocity.y < penguin.jump_impulse / 2.0 {
                    velocity.y = penguin.jump_impulse / 2.0;
                }

                transform.rotation *= Quat::from_axis_angle(Vec3::X, -PI / 2.0);
                *movement_mode = PenguinMovementMode::Sliding;
            } else {
                transform.rotation *= Quat::from_axis_angle(Vec3::X, PI / 2.0);
                *movement_mode = match grounding.is_grounded() {
                    true => PenguinMovementMode::Walking,
                    false => PenguinMovementMode::Falling,
                };
            }
        }
    }
}

fn init_ground_movement(
    trigger: Trigger<OnGroundEnter>,
    mut query: Query<&mut PenguinMovementMode>,
) {
    let mut mode = query.get_mut(trigger.target()).unwrap();
    if *mode == PenguinMovementMode::Sliding {
        return;
    }

    *mode = PenguinMovementMode::Walking;

    println!("~ WALKING");
}

fn init_air_movement(trigger: Trigger<OnGroundLeave>, mut query: Query<&mut PenguinMovementMode>) {
    let mut mode = query.get_mut(trigger.target()).unwrap();
    if *mode == PenguinMovementMode::Sliding {
        return;
    }

    *mode = PenguinMovementMode::Falling;

    println!("~ FALLING");
}

fn setup_level(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // floor
    let floor_size = 100.0;
    commands.spawn((
        RigidBody::Static,
        Collider::half_space(Vec3::Y),
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(floor_size)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(asset_server.load("texture_08.png")),
            uv_transform: Affine2::from_scale(Vec2::splat(floor_size)),
            ..Default::default()
        })),
    ));

    // camera
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: 75.0_f32.to_radians(),
            ..Default::default()
        }),
        Transform::from_xyz(0.0, 4.0, 10.0).looking_at(Vec3::ZERO, Dir3::Y),
    ));

    // light
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            ..Default::default()
        },
        Transform::from_xyz(1.0, 1.0, 1.0).looking_at(Vec3::ZERO, Dir3::Z),
    ));

    // obstacles
    let mut spawn_cube = |pos, size| {
        let cube = Cuboid::from_size(size);
        commands.spawn((
            RigidBody::Static,
            Collider::from(cube),
            Transform::from_translation(pos),
            Mesh3d(meshes.add(cube)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color_texture: Some(asset_server.load("texture_08.png")),
                ..Default::default()
            })),
        ));
    };

    spawn_cube(Vec3::new(8.0, 0.2, 0.0), Vec3::new(4.0, 0.4, 4.0));
    spawn_cube(Vec3::new(-8.0, 2.0, 0.0), Vec3::new(4.0, 4.0, 4.0));
}
