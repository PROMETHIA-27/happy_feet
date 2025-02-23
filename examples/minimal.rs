use std::{hash::Hash, ops::Range};

use avian3d::{math::PI, prelude::*};
use bevy::{input::mouse::MouseMotion, prelude::*};
use rand::{prelude::*, rng};
use slither::{
    CalculatedVelocity, Floor, MovementConfig, MovingPlatform, RotatingPlatform, move_and_slide,
    update_platform_velocity,
};

fn main() -> AppExit {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::new(FixedPostUpdate),
            PhysicsDebugPlugin::new(FixedPostUpdate),
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
        .add_systems(Update, input)
        .add_systems(
            FixedUpdate,
            (
                update_platforms,
                update_platform_velocity,
                update,
                clear_buffered_input,
            )
                .chain(),
        )
        .run()
}

const SKIN_WIDTH: f32 = 0.333;

#[derive(Component)]
#[require(Transform)]
struct CameraPole;

#[derive(Component)]
struct Player;

#[derive(Component, Default)]
struct MovementInput {
    current: Vec3,
    buffered: Vec3,
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
            MovementInput::default(),
            MovementMode::Flying,
            CharacterVelocity(Vec3::ZERO),
            RigidBody::Kinematic,
            Collider::capsule(1.0, 2.0),
            Transform {
                translation: Vec3::Y * 10.0,
                ..Default::default()
            },
            Mesh3d(meshes.add(Capsule3d::new(1.0, 2.0))),
            MeshMaterial3d(materials.add(StandardMaterial::default())),
        ))
        .with_children(|player| {
            player.spawn((
                Collider::capsule(1.0 + SKIN_WIDTH, 2.0),
                CollisionLayers::NONE,
            ));
            player
                .spawn((CameraPole, Transform {
                    translation: Vec3::Y * 2.0,
                    ..Default::default()
                }))
                .with_child((Camera3d::default(), Transform {
                    translation: Vec3::Z * 20.0,
                    ..Default::default()
                }));
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

fn input(
    mut motion: EventReader<MouseMotion>,
    key: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut query: Query<(&mut Transform, &mut MovementInput, &mut MovementMode), With<Player>>,
) {
    if !mouse.pressed(MouseButton::Left) {
        motion.clear();
    }

    let mouse_delta: Vec2 = motion.read().map(|m| m.delta).sum();

    let mut input = button_axis_3d(
        &key,
        KeyCode::KeyA..KeyCode::KeyD,
        KeyCode::KeyQ..KeyCode::KeyE,
        KeyCode::KeyW..KeyCode::KeyS,
    );

    let pressed_jump = key.just_pressed(KeyCode::Space);

    let change_mode = key.just_pressed(KeyCode::Tab);

    for (mut transform, mut movement, mut mode) in &mut query {
        if change_mode {
            *mode = match *mode {
                MovementMode::Walking => MovementMode::Flying,
                MovementMode::Flying => MovementMode::Walking,
            }
        }

        if let MovementMode::Walking = *mode {
            if pressed_jump {
                input.y = 1.0;
            } else {
                input.y = 0.0;
            }
        }
        movement.set(input);

        let right = transform.right();
        transform.rotate(Quat::from_axis_angle(*right, mouse_delta.y * -0.01));
        transform.rotate(Quat::from_rotation_y(mouse_delta.x * -0.01));
    }
}

#[derive(Component)]
struct WallNormal(Vec3);

fn update(
    mut commands: Commands,
    mut noclip_enabled: Local<bool>,
    mut gravity_enabled: Local<bool>,
    mut gizmos: Gizmos,
    spatial: SpatialQuery,
    mut query: Query<
        (
            Entity,
            &Collider,
            &mut Transform,
            &mut CharacterVelocity,
            &MovementInput,
            &MovementMode,
            Option<&Floor>,
            Option<&WallNormal>,
        ),
        With<Player>,
    >,
    platforms: Query<(&CalculatedVelocity, &Transform), Without<Player>>,
    time: Res<Time>,
    key: Res<ButtonInput<KeyCode>>,
) {
    if key.just_pressed(KeyCode::Space) {
        *gravity_enabled = !*gravity_enabled;
    }

    if key.just_pressed(KeyCode::Escape) {
        *noclip_enabled = !*noclip_enabled;
    }

    for (entity, shape, mut transform, mut velocity, input, mode, floor, wall) in &mut query {
        let inherited_velocity;
        if let Some((vel, platform_trans)) = floor.and_then(|p| platforms.get(p.entity).ok()) {
            let offset = transform.translation - platform_trans.translation;
            let tangental_vel = vel.angular.cross(offset);
            inherited_velocity = vel.linear + tangental_vel;
            // inherited_velocity = vel.linear;
        } else {
            inherited_velocity = Vec3::ZERO
        }

        let mut did_jump = false;
        {
            let vel = &mut velocity.0;

            match mode {
                MovementMode::Walking => {
                    let movement_input = Vec3::new(input.buffered.x, 0.0, input.buffered.z);
                    let jump_input = input.buffered.y > 0.0;
                    let dir = (transform.rotation * movement_input.clamp_length_max(1.0))
                        .reject_from(Vec3::Y)
                        .clamp_length_max(1.0);

                    let target_speed = 15.0;
                    let gravity = 20.0;
                    let max_acceleration;

                    let jump_dir = match wall {
                        Some(wall) if floor.is_none() => Some(wall.0),
                        _ if floor.is_some() => Some(Vec3::Y),
                        _ => None,
                    };

                    if let Some(dir) = jump_dir {
                        if jump_input {
                            let jump_height = 4.0;
                            let jump_accel = f32::sqrt(2.0 * gravity * jump_height);
                            *vel += dir * jump_accel;
                            did_jump = true;
                        }
                    }

                    if floor.is_some() {
                        // Friction
                        let len = vel.length();
                        *vel -=
                            vel.normalize_or_zero() * f32::min(len, len * 10.0 * time.delta_secs());

                        max_acceleration = 200.0;
                    } else {
                        max_acceleration = 50.0;
                        *vel -= Vec3::Y * time.delta_secs() * gravity; // Gravity
                    }

                    if let Ok(dir) = Dir3::new(dir) {
                        // Movement
                        *vel += acceleration(
                            *vel,
                            dir,
                            max_acceleration,
                            target_speed,
                            time.delta_secs(),
                        );
                    }
                }
                MovementMode::Flying => {
                    let dir = transform.rotation * input.buffered.normalize_or_zero();

                    *vel += dir * 400.0 * time.delta_secs(); // Input
                    *vel -= *vel * 20.0 * time.delta_secs(); // Friction
                }
            }
        }

        if *noclip_enabled {
            transform.translation += velocity.0 * time.delta_secs();
            continue;
        }

        let filter = SpatialQueryFilter::from_excluded_entities([entity]);

        let mut wall = None;
        let output = move_and_slide(
            floor.is_some() && !did_jump,
            transform.translation,
            velocity.0,
            transform.rotation,
            MovementConfig {
                up_direction: Dir3::Y,
                skin_width: SKIN_WIDTH,
                floor_snap_distance: 0.0,
                max_floor_angle: 45_f32.to_radians(),
            },
            shape,
            &spatial,
            &filter,
            time.delta_secs(),
            |slope| {
                if !slope.is_floor() {
                    wall = Some(*slope.normal)
                }
            },
            &mut gizmos,
        );
        transform.translation += inherited_velocity * time.delta_secs();
        transform.translation += output.motion;
        velocity.0 = output.velocity;

        if output.overlap_amount > SKIN_WIDTH {
            println!("!!! STUCK STUCK STUCK !!!");
        }

        if let Some(wall) = wall {
            commands.entity(entity).insert(WallNormal(wall));
        } else {
            commands.entity(entity).remove::<WallNormal>();
        }

        if let Some(new_floor) = output.floor {
            if floor.is_none() && !did_jump {
                println!("ON FLOOR");
                commands.entity(entity).insert(new_floor);
            }
        } else if floor.is_some() {
            println!("NOT ON FLOOR");
            velocity.0 += inherited_velocity;
            commands.entity(entity).remove::<Floor>();
        }
    }
}

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

pub fn acceleration(
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
