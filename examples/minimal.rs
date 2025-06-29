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

use happy_feet::prelude::*;

// Configure movement for ground and air states
const GROUND_MOVEMENT: CharacterMovement = CharacterMovement {
    target_speed: 8.0, // Maximum movement speed
    acceleration: 100.0,
};

const AIR_MOVEMENT: CharacterMovement = CharacterMovement {
    target_speed: 8.0,
    acceleration: 20.0,
};

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
        .add_observer(init_ground_movement)
        .add_observer(init_air_movement)
        .run()
}

fn setup_character(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let shape = Capsule3d::new(0.2, 1.0);
    commands.spawn((
        Character,
        AIR_MOVEMENT,
        // Enable stepping
        SteppingConfig {
            max_vertical: 0.3,
            ..Default::default()
        },
        // Enable gravity
        CharacterGravity::new(Vec3::Y * -20.0),
        // Enable ground friction
        GroundFriction(60.0),
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
    mut query: Query<(&mut MoveInput, &mut KinematicVelocity, &mut Grounding)>,
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

    for (mut move_input, mut velocity, mut grounding) in &mut query {
        move_input.value = move_direction;

        if grounding.is_grounded() && key.just_pressed(KeyCode::Space) {
            velocity.y = 6.0;
            grounding.detach(); // Detach from the ground to avoid snapping back to it during character update
        }
    }
}

fn init_ground_movement(trigger: Trigger<OnGroundEnter>, mut commands: Commands) {
    commands.entity(trigger.target()).insert(GROUND_MOVEMENT);
}

fn init_air_movement(trigger: Trigger<OnGroundLeave>, mut commands: Commands) {
    commands.entity(trigger.target()).insert(AIR_MOVEMENT);
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
