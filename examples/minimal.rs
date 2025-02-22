use std::{hash::Hash, ops::Range};

use avian3d::{math::PI, prelude::*};
use bevy::{input::mouse::MouseMotion, prelude::*};
use rand::{prelude::*, rng};
use slither::{
    CalculatedVelocity, MovingPlatform, RotatingPlatform, StandingOn, collide_and_slide3,
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
            (update_platforms, update_platform_velocity, update).chain(),
        )
        .run()
}

const SKIN_WIDTH: f32 = 0.2;

#[derive(Component)]
#[require(Transform)]
struct CameraArm;

#[derive(Component)]
struct Player;

fn setup_player(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn((
            Player,
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
                .spawn((CameraArm, Transform {
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

fn input(
    mut motion: EventReader<MouseMotion>,
    key: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut player: Single<(&mut Transform, &mut LinearVelocity), With<Player>>,
) {
    let (mut transform, mut velocity) = player.into_inner();

    // let input_vec = button_axis_3d(
    //     &key,
    //     KeyCode::KeyA..KeyCode::KeyD,
    //     KeyCode::KeyQ..KeyCode::KeyE,
    //     KeyCode::KeyW..KeyCode::KeyS,
    // );

    // velocity.0 = transform.rotation * input_vec.normalize_or_zero() * 10.0;

    if !mouse.pressed(MouseButton::Left) {
        motion.clear();
        return;
    }

    for motion in motion.read() {
        let right = transform.right();
        transform.rotate(Quat::from_axis_angle(*right, motion.delta.y * -0.01));
        transform.rotate(Quat::from_rotation_y(motion.delta.x * -0.01));
    }
}

#[derive(Component)]
struct CharacterVelocity(Vec3);

fn update(
    mut commands: Commands,
    mut noclip_enabled: Local<bool>,
    mut gravity_enabled: Local<bool>,
    collisions: Res<Collisions>,
    mut gizmos: Gizmos,
    spatial: SpatialQuery,
    mut query: Query<
        (
            Entity,
            &Collider,
            &mut Transform,
            &mut CharacterVelocity,
            Option<&StandingOn>,
        ),
        With<Player>,
    >,
    platforms: Query<(&CalculatedVelocity, &Transform), Without<Player>>,
    time: Res<Time>,
    key: Res<ButtonInput<KeyCode>>,
) {
    let input = button_axis_3d(
        &key,
        KeyCode::KeyA..KeyCode::KeyD,
        KeyCode::KeyQ..KeyCode::KeyE,
        KeyCode::KeyW..KeyCode::KeyS,
    )
    .normalize_or_zero();

    if key.just_pressed(KeyCode::Space) {
        *gravity_enabled = !*gravity_enabled;
    }

    if key.just_pressed(KeyCode::Escape) {
        *noclip_enabled = !*noclip_enabled;
    }

    for (entity, shape, mut transform, mut velocity, standing_on) in &mut query {
        let inherited_velocity;
        if let Some((vel, platform_trans)) = standing_on.and_then(|p| platforms.get(p.0).ok()) {
            let offset = transform.translation - platform_trans.translation;
            let tangental_vel = vel.angular.cross(offset);
            inherited_velocity = vel.linear + tangental_vel;
        } else {
            inherited_velocity = Vec3::ZERO
        }

        if inherited_velocity.length_squared() > 0.01 {
            dbg!(inherited_velocity);
        }

        if *gravity_enabled {
            velocity.0 += (transform.rotation * input)
                .reject_from(Vec3::Y)
                .normalize_or_zero()
                * 100.0
                * time.delta_secs();
            velocity.0 += Vec3::NEG_Y * time.delta_secs() * 10.0;
        } else {
            let vel = &mut velocity.0;
            *vel += transform.rotation * input * 400.0 * time.delta_secs();
            *vel -= *vel * 20.0 * time.delta_secs();
        }

        if *noclip_enabled {
            transform.translation += velocity.0 * time.delta_secs();
            continue;
        }

        transform.translation += inherited_velocity * time.delta_secs();

        let filter = SpatialQueryFilter::from_excluded_entities([entity]);
        // let mut velocities = [velocity.0];
        let out = collide_and_slide3(
            transform.translation,
            transform.rotation,
            velocity.0,
            Dir3::Y,
            SKIN_WIDTH,
            60_f32.to_radians(),
            0.0,
            0.5,
            0.0,
            false,
            true,
            shape,
            &filter,
            &spatial,
            time.delta_secs(),
            &mut gizmos,
        );

        let delta = transform.translation - out.position;
        transform.translation = out.position;
        velocity.0 = out.velocity;

        if let Some(e) = out.floor_entity {
            commands.entity(entity).insert(StandingOn(e));
        } else if standing_on.is_some() {
            commands.entity(entity).remove::<StandingOn>();
        }

        // dbg!(delta.reject_from(Vec3::Y).length());

        // let position = collide_and_slide2(
        //     transform.translation,
        //     transform.rotation,
        //     velocity.0 * time.delta_secs(),
        //     &CollideAndSlideConfig {
        //         skin_width: CharacterValue::Relative(0.2),
        //         // skin_width: CharacterValue::Relative(0.01),
        //         // floor_snap_distance: CharacterValue::Relative(1.0),
        //         floor_snap_distance: CharacterValue::ZERO,
        //         max_step_height: CharacterValue::ZERO,
        //         max_slope_angle: 60_f32.to_radians(),
        //         apply_remaining_motion: true,
        //         ..Default::default()
        //     },
        //     shape,
        //     &filter,
        //     &spatial,
        //     |normal| {
        //         // velocity.0 =
        //         //     velocity.0.reject_from(normal).normalize_or_zero() * velocity.0.length();
        //         velocity.0 = velocity.0.reject_from(normal);
        //     },
        //     &mut gizmos,
        // );
        // transform.translation = position;
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

fn button_axis_2d<T>(input: &ButtonInput<T>, x: Range<T>, y: Range<T>) -> Vec2
where
    T: Hash + Eq + Copy + Send + Sync + 'static,
{
    Vec2 {
        x: button_axis(input, x),
        y: button_axis(input, y),
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
