use avian3d::prelude::*;
use bevy::{color::palettes::css::*, input::InputSystem, prelude::*};
use happy_feet::prelude::*;

const GROUND_MOVEMENT: CharacterMovement = CharacterMovement {
    target_speed: 8.0,
    acceleration: 100.0,
};

// Less acceleration when in the air
const AIR_MOVEMENT: CharacterMovement = CharacterMovement {
    target_speed: 8.0,
    acceleration: 20.0,
};

fn main() -> AppExit {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            CharacterPlugin::default(),
        ))
        .add_systems(Startup, (setup_character, setup_level))
        .add_systems(PreUpdate, character_input.after(InputSystem))
        .add_observer(on_enter_ground)
        .add_observer(on_leave_ground)
        .run()
}

fn on_enter_ground(trigger: Trigger<OnGroundEnter>, mut commands: Commands) {
    commands.entity(trigger.target()).insert(GROUND_MOVEMENT);
}

fn on_leave_ground(trigger: Trigger<OnGroundLeave>, mut commands: Commands) {
    commands.entity(trigger.target()).insert(AIR_MOVEMENT);
}

fn setup_character(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let shape = Capsule3d::new(0.2, 1.0);
    commands.spawn((
        Character,
        CharacterGravity(Vec3::NEG_Y * 20.0),
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
    // movement
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
        move_input.set(move_direction);

        // jump
        if grounding.is_grounded() && key.just_pressed(KeyCode::Space) {
            // It's important to detach from the ground since otherwise the character will just instantly snap back after jumping.
            grounding.detach();

            velocity.y = 6.0;
        }
    }
}

fn setup_level(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // floor
    commands.spawn((
        RigidBody::Static,
        Collider::half_space(Vec3::Y),
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(100.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(asset_server.load("texture_08.png")),
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
