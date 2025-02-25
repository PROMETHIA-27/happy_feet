use std::{hash::Hash, ops::Range};

use avian3d::{math::PI, prelude::*};
use bevy::{input::mouse::MouseMotion, prelude::*, render::camera::CameraProjection};
use rand::{prelude::*, rng};
use slither::{
    CalculatedVelocity, Character, CharacterMovementPlugin, CharacterMovementSystems, Floor,
    MovementConfig, MovingPlatform, RotatingPlatform, SlopePlane, move_and_slide, project_on_floor,
    update_platform_velocity,
};

fn main() -> AppExit {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::new(FixedPostUpdate),
            PhysicsDebugPlugin::new(FixedPostUpdate),
            CharacterMovementPlugin,
        ))
        .add_systems(Startup, setup_player)
        .add_systems(
            Startup,
            (
                setup_level,
                setup_stairs,
                setup_slopes,
                setup_rubble,
                setup_platforms,
            ),
        )
        .add_systems(Update, update_input)
        .add_systems(
            FixedUpdate,
            (
                update_platforms,
                update_platform_velocity,
                update_movement,
                clear_buffered_input,
            )
                .before(CharacterMovementSystems)
                .chain(),
        )
        .run()
}

const SKIN_WIDTH: f32 = 0.2;

#[derive(Component)]
#[require(Transform)]
struct CameraArm;

#[derive(Component)]
struct Player;

#[derive(Component, Default)]
struct MovementInput {
    current: Vec3,
    buffered: Vec3,
    is_sprinting: bool,
}

impl MovementInput {
    fn set(&mut self, value: Vec3) {
        self.buffered += value;
        self.current = value;
    }
}

#[derive(Component)]
struct CharacterVelocity(Vec3);

#[derive(Component)]
enum MovementMode {
    Walking,
    Flying,
}

fn setup_player(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn((
            Player,
            Character {
                skin_width: SKIN_WIDTH,
                climb_up_walls: false,
                ..Default::default()
            },
            MovementInput::default(),
            MovementMode::Flying,
            CharacterVelocity(Vec3::ZERO),
            (
                RigidBody::Kinematic,
                GravityScale(0.0),
                LockedAxes::ROTATION_LOCKED,
            ),
            Collider::capsule(1.0, 2.0),
            Transform {
                translation: Vec3::Y * 10.0,
                ..Default::default()
            },
            // Mesh3d(meshes.add(Capsule3d::new(1.0, 2.0))),
            // MeshMaterial3d(materials.add(StandardMaterial::default())),
        ))
        .with_children(|player| {
            player.spawn((
                Collider::capsule(1.0 + SKIN_WIDTH, 2.0),
                CollisionLayers::NONE,
            ));
            player
                .spawn((
                    CameraArm,
                    Transform {
                        translation: Vec3::Y * 2.0,
                        ..Default::default()
                    },
                ))
                .with_child((
                    Camera3d::default(),
                    Projection::Perspective(PerspectiveProjection {
                        fov: 80_f32.to_radians(),
                        ..Default::default()
                    }),
                    Transform {
                        translation: Vec3::Z * 10.0, // Camera distance.
                        ..Default::default()
                    },
                ));
        });
}

fn setup_slopes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cuboid = Cuboid::from_length(1.0);
    let mesh = meshes.add(cuboid);
    let material = materials.add(StandardMaterial::default());

    let height = 40.0;
    let width = 10.0;
    let origin = Vec3::new(-width * 2.5, -height / 2.0, -30.0);

    for i in 1..5 {
        let angle = i as f32 / PI;
        let rotation = Quat::from_axis_angle(Vec3::X, angle);
        let offset = Vec3::X * i as f32 * width;
        commands.spawn((
            Transform {
                translation: origin + offset,
                rotation,
                scale: Vec3::new(width, height, height),
                ..Default::default()
            },
            RigidBody::Static,
            Collider::from(cuboid),
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material.clone()),
        ));
    }
}

fn setup_stairs(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cuboid = Cuboid::from_length(1.0);
    let mesh = meshes.add(cuboid);
    let material = materials.add(StandardMaterial::default());

    let depth = 4.0;
    let width = 30.0;
    let height_factor = 0.2;
    let origin = Vec3::new(20.0 - depth, 0.0, 0.0);
    for i in 1..9 {
        let height = i as f32 * i as f32 * height_factor;
        let offset = Vec3::X * i as f32 * depth + Vec3::Y * height / 2.0;
        commands.spawn((
            Transform {
                translation: origin + offset,
                scale: Vec3::new(depth, height, width),
                ..Default::default()
            },
            RigidBody::Static,
            Collider::from(cuboid),
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material.clone()),
        ));
    }
}

fn setup_rubble(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut rng = rng();
    let cuboid = Cuboid::from_length(1.0);
    let mesh = meshes.add(cuboid);
    let material = materials.add(StandardMaterial::default());

    let origin = Vec3::new(-50.0, 0.0, 0.0);
    for _ in 0..200 {
        let dist = rng.random_range(0.0_f32..1.0_f32).powi(2) * 30.0;
        let angle = rng.random_range(-PI..PI);
        let offset = Quat::from_rotation_y(angle) * (Vec3::X * dist);
        let rotation = Quat::from_euler(
            EulerRot::XYZ,
            rng.random_range(-PI..PI),
            rng.random_range(-PI..PI),
            rng.random_range(-PI..PI),
        );
        let size = rng.random_range(0.1..10.0);
        commands.spawn((
            Transform {
                translation: origin + offset,
                rotation,
                scale: Vec3::splat(size),
            },
            RigidBody::Static,
            Collider::from(cuboid),
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material.clone()),
        ));
    }
}

fn setup_platforms(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cuboid = Cuboid::from_length(1.0);
    let mesh = meshes.add(cuboid);
    let material = materials.add(StandardMaterial::default());

    let height = 1.0;
    let width = 10.0;
    let origin = Vec3::new(0.0, 0.0, 25.0);
    commands.spawn((
        MovingPlatform {
            start: origin,
            end: origin + Vec3::Y * 10.0,
            speed: 1.0,
        },
        RotatingPlatform {
            axis: Dir3::Y,
            speed: 1.0,
        },
        Transform {
            translation: origin,
            scale: Vec3::new(width, height, width),
            ..Default::default()
        },
        RigidBody::Static,
        Collider::from(cuboid),
        Mesh3d(mesh.clone()),
        MeshMaterial3d(material.clone()),
    ));
}

fn setup_level(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        DirectionalLight {
            illuminance: light_consts::lux::DARK_OVERCAST_DAY,
            shadows_enabled: true,
            ..Default::default()
        },
        Transform {
            rotation: Quat::from_rotation_x(-PI / 2.0),
            ..Default::default()
        },
    ));

    commands.spawn((
        RigidBody::Static,
        Collider::half_space(Vec3::Y),
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(100.0)))),
        MeshMaterial3d(materials.add(StandardMaterial::default())),
    ));
}

fn update_platforms(
    mut query: Query<(&mut Transform, AnyOf<(&MovingPlatform, &RotatingPlatform)>)>,
    time: Res<Time>,
) {
    for (mut transform, platform) in &mut query {
        if let Some(platform) = platform.0 {
            let pos = platform.start.lerp(
                platform.end,
                (time.elapsed_secs() * platform.speed).sin() / 2.0 + 0.5,
            );
            transform.translation = pos;
        }
        if let Some(platform) = platform.1 {
            transform.rotate_axis(platform.axis, platform.speed * time.delta_secs());
        }
    }
}

fn update_input(
    mut motion: EventReader<MouseMotion>,
    key: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut players: Query<(&mut Transform, &mut MovementInput, &mut MovementMode), With<Player>>,
    mut cameras: Query<&mut Transform, (With<CameraArm>, Without<Player>)>,
) {
    if !mouse.pressed(MouseButton::Left) {
        motion.clear();
    }

    let mouse_delta: Vec2 = motion.read().map(|m| m.delta).sum();

    let mut move_axis = button_axis_3d(
        &key,
        KeyCode::KeyA..KeyCode::KeyD,
        KeyCode::KeyQ..KeyCode::KeyE,
        KeyCode::KeyW..KeyCode::KeyS,
    );

    let pressed_jump = key.just_pressed(KeyCode::Space);
    let holding_sprint = key.pressed(KeyCode::ShiftLeft);
    let change_mode = key.just_pressed(KeyCode::Tab);

    for (mut transform, mut input, mut mode) in &mut players {
        input.is_sprinting = holding_sprint;

        if change_mode {
            *mode = match *mode {
                MovementMode::Walking => MovementMode::Flying,
                MovementMode::Flying => MovementMode::Walking,
            }
        }

        if let MovementMode::Walking = *mode {
            if pressed_jump {
                move_axis.y = 1.0;
            } else {
                move_axis.y = 0.0;
            }
        }

        input.set(move_axis);

        // let right = transform.right();
        // transform.rotate(Quat::from_axis_angle(*right, mouse_delta.y * -0.01));
        transform.rotate(Quat::from_rotation_y(mouse_delta.x * -0.01));
    }

    for mut transform in &mut cameras {
        let right = transform.right();
        transform.rotate(Quat::from_axis_angle(*right, mouse_delta.y * -0.01));
        // transform.rotate(Quat::from_rotation_y(mouse_delta.x * -0.01));
    }
}

fn update_movement(
    mut query: Query<
        (
            &mut Character,
            &mut LinearVelocity,
            &MovementInput,
            &MovementMode,
            &Transform,
        ),
        With<Player>,
    >,
    time: Res<Time>,
) {
    const GRAVITY: f32 = 21.0;

    for (mut character, mut _velocity, input, mode, transform) in &mut query {
        let target_speed = match input.is_sprinting {
            true => 35.0,
            false => 15.0,
        };

        match mode {
            MovementMode::Walking => {
                if input.buffered.y > 0.0 {
                    let jump_height = 4.0;
                    let jump_accel = f32::sqrt(2.0 * GRAVITY * jump_height) * Vec3::Y;

                    character.velocity += jump_accel;

                    character.floor = None;
                }

                if let Ok(direction) =
                    Dir3::new((transform.rotation * input.buffered).reject_from_normalized(Vec3::Y))
                {
                    let max_acceleration = match character.floor.is_some() {
                        true => 400.0,
                        false => 40.0,
                    };

                    let accel = acceleration(
                        character.velocity,
                        direction,
                        max_acceleration,
                        target_speed,
                        time.delta_secs(),
                    );

                    if let Some(floor) = character.floor {
                        let accel = project_on_floor(accel, floor.normal, Dir3::Y, true);
                        character.velocity += accel;
                    } else {
                        character.velocity += accel;
                    }
                }

                if character.floor.is_some() {
                    let fric = friction(character.velocity, 10.0, time.delta_secs());
                    character.velocity += fric;
                } else {
                    character.velocity -= GRAVITY * Vec3::Y * time.delta_secs();
                }
            }
            MovementMode::Flying => {
                if let Ok(direction) = Dir3::new(transform.rotation * input.buffered) {
                    let accel = acceleration(
                        character.velocity,
                        direction,
                        400.0,
                        target_speed,
                        time.delta_secs(),
                    );
                    character.velocity += accel;
                }

                let fric = friction(character.velocity, 10.0, time.delta_secs());
                character.velocity += fric;
            }
        }
    }
}

#[derive(Component)]
struct WallNormal(Dir3);

fn clear_buffered_input(mut query: Query<&mut MovementInput>) {
    for mut input in &mut query {
        input.buffered = Vec3::ZERO;
    }
}

fn button_axis<T>(input: &ButtonInput<T>, range: Range<T>) -> f32
where
    T: Hash + Eq + Copy + Send + Sync + 'static,
{
    match (input.pressed(range.start), input.pressed(range.end)) {
        (true, false) => -1.0,
        (false, true) => 1.0,
        _ => 0.0,
    }
}

fn button_axis_3d<T>(input: &ButtonInput<T>, x: Range<T>, y: Range<T>, z: Range<T>) -> Vec3
where
    T: Hash + Eq + Copy + Send + Sync + 'static,
{
    Vec3 {
        x: button_axis(input, x),
        y: button_axis(input, y),
        z: button_axis(input, z),
    }
}

fn acceleration(
    velocity: Vec3,
    direction: Dir3,
    max_acceleration: f32,
    target_speed: f32,
    delta: f32,
) -> Vec3 {
    // Current speed in the desired direction.
    let current_speed = velocity.dot(*direction);

    // No acceleration is needed if current speed exceeds target.
    if current_speed >= target_speed {
        return Vec3::ZERO;
    }
    // Clamp to avoid acceleration past the target speed.
    let accel_speed = f32::min(target_speed - current_speed, max_acceleration * delta);
    accel_speed * direction
}

fn friction(velocity: Vec3, friction: f32, delta: f32) -> Vec3 {
    let length = velocity.length();
    -velocity.normalize_or_zero() * f32::min(length, length * friction * delta)
}
