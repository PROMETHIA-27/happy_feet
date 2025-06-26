use avian3d::prelude::*;
use bevy::{
    color::palettes::css::*,
    image::{ImageAddressMode, ImageSamplerDescriptor},
    input::InputSystem,
    math::Affine2,
    prelude::*,
};

use happy_feet::character::{KinematicVelocity, Projectile};

fn main() -> AppExit {
    App::new()
        .add_plugins((
            DefaultPlugins.set(ImagePlugin {
                default_sampler: ImageSamplerDescriptor {
                    address_mode_u: ImageAddressMode::Repeat,
                    address_mode_v: ImageAddressMode::Repeat,
                    ..Default::default()
                },
            }),
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            // CharacterPlugin::default(),
            happy_feet::character::CharacterPlugin,
        ))
        .add_systems(Startup, (setup_character, setup_level))
        .add_systems(PreUpdate, character_input.after(InputSystem))
        .run()
}

fn setup_character(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let shape = Capsule3d::new(0.2, 1.0);
    commands.spawn((
        // Character::default(),
        // CharacterGravity(Vec3::NEG_Y * 20.0),
        // CharacterFriction(40.0),
        // Transform::from_xyz(0.0, 1.0, 0.0),
        Collider::from(shape),
        Mesh3d(meshes.add(shape)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: MIDNIGHT_BLUE.into(),
            perceptual_roughness: 0.8,
            ..Default::default()
        })),
        Projectile,
        // CollideAndSlideConfig::default(),
        // GroundingConfig {
        //     max_distance: 10.0,
        //     ..Default::default()
        // },
        // SteppingConfig {
        //     max_vertical: 10.0,
        //     ..Default::default()
        // },
        // SteppingBehaviour::Always,
    ));
}

fn character_input(key: Res<ButtonInput<KeyCode>>, mut query: Query<&mut KinematicVelocity>) {
    // movement
    let x = match (key.pressed(KeyCode::KeyA), key.pressed(KeyCode::KeyD)) {
        (true, false) => -1.0,
        (false, true) => 1.0,
        _ => 0.0,
    };
    let y = match (key.pressed(KeyCode::KeyQ), key.pressed(KeyCode::KeyE)) {
        (true, false) => -1.0,
        (false, true) => 1.0,
        _ => 0.0,
    };
    let z = match (key.pressed(KeyCode::KeyW), key.pressed(KeyCode::KeyS)) {
        (true, false) => -1.0,
        (false, true) => 1.0,
        _ => 0.0,
    };

    let move_direction = Vec3::new(x, y, z).normalize_or_zero();

    for mut velocity in &mut query {
        velocity.0 = move_direction * 8.0;
    }
}

fn setup_level(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // box
    let cube = Cuboid::from_length(2.0);
    commands.spawn((
        RigidBody::Static,
        Collider::from(cube),
        Transform::from_xyz(4.0, 1.0, 0.0),
        Mesh3d(meshes.add(cube)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(asset_server.load("texture_08.png")),
            ..Default::default()
        })),
    ));

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
}
