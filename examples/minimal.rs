use std::{hash::Hash, ops::Range};

use avian3d::{math::PI, prelude::*};
use bevy::{input::mouse::MouseMotion, math::primitives, prelude::*};
use rand::{prelude::*, rng};
use slither::{CollideAndSlideConfig, SkinWidth, collide_and_slide, collide_and_slide2};

fn main() -> AppExit {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::new(FixedPostUpdate),
            PhysicsDebugPlugin::new(FixedPostUpdate),
        ))
        .add_systems(Startup, (setup_player, setup_level))
        .add_systems(Update, input)
        .add_systems(FixedUpdate, update)
        .run()
}

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

    commands.spawn((
        Transform {
            translation: Vec3::new(50.0, 0.0, 50.0),
            ..Default::default()
        },
        RigidBody::Static,
        Collider::sphere(25.0),
        Mesh3d(meshes.add(Sphere::new(25.0))),
        MeshMaterial3d(materials.add(StandardMaterial::default())),
    ));

    let mut rng = rng();
    let cuboid = Cuboid::from_length(1.0);
    let mesh = meshes.add(cuboid);
    let material = materials.add(StandardMaterial::default());
    for _ in 0..200 {
        let offset = Vec3::new(
            rng.random_range(-20.0..20.0),
            0.0,
            rng.random_range(-20.0..20.0),
        );
        let rotation = Quat::from_euler(
            EulerRot::XYZ,
            rng.random_range(-PI..PI),
            rng.random_range(-PI..PI),
            rng.random_range(-PI..PI),
        );
        let size = rng.random_range(0.1..10.0);
        commands.spawn((
            Transform {
                translation: offset,
                rotation,
                scale: Vec3::splat(size),
                ..Default::default()
            },
            RigidBody::Static,
            Collider::from(cuboid),
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material.clone()),
        ));
    }
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
    mut noclip_enabled: Local<bool>,
    mut gravity_enabled: Local<bool>,
    collisions: Res<Collisions>,
    mut gizmos: Gizmos,
    spatial: SpatialQuery,
    mut query: Query<(Entity, &Collider, &mut Transform, &mut CharacterVelocity), With<Player>>,
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

    for (entity, shape, mut transform, mut velocity) in &mut query {
        let collisions = collisions.collisions_with_entity(entity);

        if *gravity_enabled {
            velocity.0 += (transform.rotation * input)
                .reject_from(Vec3::Y)
                .normalize_or_zero()
                * 10.0
                * time.delta_secs();
            velocity.0 += Vec3::NEG_Y * time.delta_secs() * 10.0;
        } else {
            velocity.0 = transform.rotation * input * 10.0;
        }

        if *noclip_enabled {
            transform.translation += velocity.0 * time.delta_secs();
            continue;
        }

        let filter = SpatialQueryFilter::from_excluded_entities([entity]);
        let position = collide_and_slide2(
            transform.translation,
            transform.rotation,
            velocity.0 * time.delta_secs(),
            &CollideAndSlideConfig {
                skin_width: SkinWidth::Relative(0.1),
                apply_remaining_motion: true,
                ..Default::default()
            },
            shape,
            &filter,
            &spatial,
            |hit| {
                velocity.0 = velocity.0.reject_from(hit.normal1);
            },
            &mut gizmos,
        );
        // velocity.0 = Vec3::ZERO;
        transform.translation = position;
        // transform.translation += out.motion + out.remaining;
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
