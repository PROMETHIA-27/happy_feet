use std::{f32::consts::PI, fmt::DebugMap};

use avian3d::prelude::*;
use bevy::{
    color::palettes::css::*,
    prelude::*,
    window::{CursorGrabMode, PrimaryWindow},
};
use bevy_enhanced_input::prelude::*;
use bevy_skein::SkeinPlugin;
use happy_feet::{
    Character, CharacterMovement, GroundingSettings, KinematicCharacterPlugin, MoveInput,
    OnGroundEnter, OnGroundLeave, SteppingBehaviour, SteppingSettings,
    debug::{DebugInput, DebugMode, DebugMotion},
};

fn main() -> AppExit {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            // PhysicsDebugPlugin::default(),
            SkeinPlugin::default(),
            KinematicCharacterPlugin,
            EnhancedInputPlugin,
        ))
        .insert_resource(AmbientLight {
            color: ALICE_BLUE.into(),
            brightness: 200.0,
            ..Default::default()
        })
        .init_gizmo_group::<PhysicsGizmos>()
        .add_input_context::<Walking>()
        .add_observer(on_collision_events_start)
        .add_observer(on_collision_events_end)
        .add_observer(on_ground_enter)
        .add_observer(on_ground_leave)
        .add_observer(on_jump)
        .add_observer(on_toggle_perspective)
        .add_observer(on_toggle_debug_mode)
        .add_observer(on_toggle_fly_mode)
        .add_systems(Startup, setup)
        .add_systems(
            PreUpdate,
            (move_input, look_input).after(EnhancedInputSystem),
        )
        .add_systems(Update, (remove_ground_when_flying, capture_mouse))
        .add_systems(
            PostUpdate,
            (
                update_attachments.after(TransformSystem::TransformPropagate),
                update_camera_offset,
                sync_attachment_global_transforms,
            )
                .chain(),
        )
        .run()
}

fn capture_mouse(mut query: Query<&mut Window, Added<PrimaryWindow>>) {
    for mut window in &mut query {
        window.cursor_options.grab_mode = CursorGrabMode::Locked;
        window.cursor_options.visible = false;
    }
}

#[derive(Component, Reflect, Default, Debug)]
#[reflect(Component)]
struct PlayerCamera {
    eye_height: f32,
    distance: f32,
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
#[relationship_target(relationship = AttachedTo)]
struct Attachments(Vec<Entity>);

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
#[relationship(relationship_target = Attachments)]
#[require(AttachmentPosition)]
struct AttachedTo(Entity);

#[derive(Component, Reflect, Default, Debug)]
#[reflect(Component)]
struct AttachmentPosition(Vec3);

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
enum MovementMode {
    Walking,
    Flying,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // let shape = Capsule3d::new(0.4, 1.0);
    // let shape = Cuboid::from_length(0.4);
    // let shape = Cone::new(0.4, 1.4);
    let shape = Capsule3d::new(0.2, 1.0);
    // let shape = Cylinder::new(0.2, 1.0);

    commands.spawn((
        Name::new("Player"),
        MovementMode::Walking,
        CollisionEventsEnabled,
        // Sensor,
        DebugMotion::default(),
        DebugInput,
        Character {
            skin_width: 0.1,
            ..Default::default()
        },
        SteppingSettings {
            max_height: 0.4,
            behaviour: SteppingBehaviour::Always,
            ..Default::default()
        },
        GroundingSettings {
            max_angle: PI / 4.0 + 0.1,
            max_distance: 0.2,
            ..Default::default()
        },
        walking_actions(),
        Mass(10.0),
        Collider::from(shape),
        Mesh3d(meshes.add(shape)),
        MeshMaterial3d(materials.add(StandardMaterial {
            alpha_mode: AlphaMode::Blend,
            base_color: WHITE.with_alpha(0.5).into(),
            ..Default::default()
        })),
        Transform {
            translation: Vec3::new(0.0, 10.0, 0.0),
            rotation: Quat::from_rotation_x(PI),
            ..Default::default()
        },
        Attachments::spawn_one((
            PlayerCamera {
                eye_height: 0.5,
                ..Default::default()
            },
            Projection::Perspective(PerspectiveProjection {
                fov: 75.0_f32.to_radians(),
                ..Default::default()
            }),
            Transform::from_xyz(0.0, 1.0, 10.0),
            Camera3d::default(),
        )),
    ));

    commands.spawn(SceneRoot(
        asset_server.load(GltfAssetLabel::Scene(0).from_asset("playground.gltf")),
    ));
}

#[derive(InputContext)]
struct Walking;

#[derive(InputAction, Debug)]
#[input_action(output = Vec3)]
struct Move;

#[derive(InputAction, Debug)]
#[input_action(output = bool)]
struct Jump;

#[derive(InputAction, Debug)]
#[input_action(output = Vec2)]
struct Look;

#[derive(InputAction, Debug)]
#[input_action(output = bool)]
struct TogglePerspective;

#[derive(InputAction, Debug)]
#[input_action(output = bool)]
struct ToggleDebugMode;

#[derive(InputAction, Debug)]
#[input_action(output = bool)]
struct ToggleFlyMode;

fn walking_actions() -> Actions<Walking> {
    let mut actions = Actions::default();

    actions
        .bind::<Move>()
        .to((
            Cardinal::wasd_keys(),
            Bidirectional {
                positive: KeyCode::KeyE.with_modifiers(SwizzleAxis::YZX),
                negative: KeyCode::KeyQ.with_modifiers(SwizzleAxis::YZX),
            },
        ))
        .with_modifiers((Negate::y(), SwizzleAxis::XZY));

    actions
        .bind::<Jump>()
        .to(KeyCode::Space)
        .with_conditions(JustPress::default());

    actions
        .bind::<Look>()
        .to(Input::mouse_motion())
        .with_modifiers((Scale::splat(-0.01), SwizzleAxis::YXZ));

    actions
        .bind::<ToggleFlyMode>()
        .to(KeyCode::KeyF)
        .with_conditions(JustPress::default());

    actions
        .bind::<TogglePerspective>()
        .to(KeyCode::KeyC)
        .with_conditions(JustPress::default());

    actions
        .bind::<ToggleDebugMode>()
        .to(KeyCode::Tab)
        .with_conditions(JustPress::default());

    actions
}

fn on_toggle_fly_mode(
    trigger: Trigger<Fired<ToggleFlyMode>>,
    mut query: Query<(&mut MovementMode, &mut CharacterMovement)>,
) {
    let (mut mode, mut movement) = query.get_mut(trigger.target()).unwrap();
    match *mode {
        MovementMode::Walking => {
            *mode = MovementMode::Flying;

            movement.drag = 10.0;
            movement.air_acceleratin = 60.0;
            movement.gravity = Vec3::ZERO;
        }
        MovementMode::Flying => {
            *mode = MovementMode::Walking;

            movement.drag = 0.01;
            movement.air_acceleratin = 20.0;
            movement.gravity = Vec3::NEG_Y * 20.0;
        }
    }
}

fn on_toggle_debug_mode(
    trigger: Trigger<Fired<ToggleDebugMode>>,
    mut commands: Commands,
    debug_modes: Query<Has<DebugMode>>,
) {
    match debug_modes.get(trigger.target()).unwrap() {
        true => commands
            .entity(trigger.target())
            .remove::<DebugMode>()
            .insert(DebugMotion::default()),
        false => commands
            .entity(trigger.target())
            .insert((DebugMode, DebugMotion::default())),
    };
}

fn on_toggle_perspective(
    trigger: Trigger<Fired<TogglePerspective>>,
    targets: Query<&Attachments>,
    mut cameras: Query<&mut PlayerCamera>,
) -> Result {
    let attachments = targets.get(trigger.target())?;

    let mut iter = cameras.iter_many_mut(attachments.iter());

    while let Some(mut player_camera) = iter.fetch_next() {
        match player_camera.distance > 0.0 {
            true => player_camera.distance = 0.0,
            false => player_camera.distance = 8.0,
        }
    }

    Ok(())
}

fn on_jump(
    trigger: Trigger<Fired<Jump>>,
    mut query: Query<(&mut Character, &CharacterMovement, &MovementMode)>,
) -> Result {
    let (mut character, movement, mode) = query.get_mut(trigger.target())?;
    if let MovementMode::Walking = mode {
        character.jump(movement.jump_impulse);
    }
    Ok(())
}

fn remove_ground_when_flying(mut query: Query<(&mut Character, &MovementMode)>) {
    for (mut character, mode) in &mut query {
        if let MovementMode::Flying = mode {
            // character.ground = None;
        }
    }
}

fn move_input(
    mut query: Query<(&mut MoveInput, &mut MovementMode, &Actions<Walking>)>,
    camera: Single<&GlobalTransform, With<PlayerCamera>>,
) {
    let mut camera_transform = camera.compute_transform();

    for (mut input, mode, actions) in &mut query {
        if let MovementMode::Walking = *mode {
            camera_transform.align(Dir3::Y, Dir3::Y, Dir3::NEG_Z, camera_transform.forward());
        }

        let axis = actions.action::<Move>().value().as_axis3d();

        input.set(camera_transform.rotation * axis.normalize_or_zero());
    }
}

fn look_input(
    characters: Query<&Actions<Walking>>,
    mut cameras: Query<(&mut Transform, &Projection, &AttachedTo)>,
) -> Result {
    for (mut camera_transform, projection, attached_to) in &mut cameras {
        let &Projection::Perspective(PerspectiveProjection { fov, .. }) = projection else {
            Err("expected perspective projection")?
        };

        let actions = characters.get(attached_to.0)?;

        let axis = actions.action::<Look>().value().as_axis2d() / fov;

        let (mut yaw, mut pitch, roll) = camera_transform.rotation.to_euler(EulerRot::YXZ);

        let max_pitch = PI / 2.0 - 1e-4;
        pitch = (pitch + axis.x).clamp(-max_pitch, max_pitch);
        yaw += axis.y;

        let rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll);

        camera_transform.rotation = rotation;
    }

    Ok(())
}

fn update_camera_offset(
    characters: Query<&Character>,
    mut cameras: Query<(&mut Transform, &PlayerCamera, &AttachedTo)>,
) -> Result {
    for (mut camera_transform, player_camera, attached_to) in &mut cameras {
        let character = characters.get(attached_to.0)?;

        let mut offset = character.up * player_camera.eye_height;
        offset += camera_transform.rotation * Vec3::Z * player_camera.distance;

        camera_transform.translation += offset;
    }

    Ok(())
}

fn update_attachments(
    mut query: Query<(&mut Transform, &AttachedTo)>,
    targets: Query<&GlobalTransform, Without<AttachedTo>>,
) -> Result {
    for (mut transform, attached_to) in &mut query {
        let target_transform = targets.get(attached_to.0)?;
        transform.translation = target_transform.translation();
    }

    Ok(())
}

fn sync_attachment_global_transforms(
    transform_helper: TransformHelper,
    mut query: Query<(Entity, &mut GlobalTransform), With<AttachedTo>>,
) -> Result {
    for (entity, mut transform) in &mut query {
        *transform = transform_helper.compute_global_transform(entity)?;
    }

    Ok(())
}

fn on_collision_events_start(
    trigger: Trigger<OnCollisionStart>,
    query: Query<Entity, With<Character>>,
    names: Query<NameOrEntity>,
) {
    if !query.contains(trigger.target()) {
        return;
    }

    info!(
        "COLLISION STARTED: {} <-> {}",
        names.get(trigger.target()).unwrap(),
        names.get(trigger.collider).unwrap(),
    );
}

fn on_collision_events_end(
    trigger: Trigger<OnCollisionEnd>,
    query: Query<Entity, With<Character>>,
    names: Query<NameOrEntity>,
) {
    if !query.contains(trigger.target()) {
        return;
    }

    info!(
        "COLLISION ENDED: {} <-> {}",
        names.get(trigger.target()).unwrap(),
        names.get(trigger.collider).unwrap()
    );
}

fn on_ground_enter(_: Trigger<OnGroundEnter>) {
    info!("ENTERED GROUND");
}

fn on_ground_leave(_: Trigger<OnGroundLeave>) {
    info!("LEFT GROUND");
}
