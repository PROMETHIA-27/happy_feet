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
    Character, CharacterDebugMode, CharacterMovement, KinematicCharacterPlugin, MoveInput,
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
        .add_input_context::<OnFoot>()
        .add_observer(on_jump)
        .add_observer(on_toggle_perspective)
        .add_observer(on_toggle_debug_mode)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                move_input,
                look_input,
                update_attachments,
                update_camera_offset.after(update_attachments),
                capture_mouse,
            ),
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

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // let shape = Capsule3d::new(0.4, 1.0);
    // let shape = Cone::new(0.4, 1.4);
    let shape = Capsule3d::new(0.2, 1.0);

    commands.spawn((
        Name::new("Player"),
        Character {
            skin_width: 0.1,
            walkable_angle: PI / 4.0 + 0.1,
            ..Default::default()
        },
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
        default_actions(),
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
struct OnFoot;

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

fn default_actions() -> Actions<OnFoot> {
    let mut actions = Actions::default();

    actions
        .bind::<Move>()
        .to(Cardinal::wasd_keys())
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
        .bind::<TogglePerspective>()
        .to(KeyCode::KeyC)
        .with_conditions(JustPress::default());

    actions
        .bind::<ToggleDebugMode>()
        .to(KeyCode::Tab)
        .with_conditions(JustPress::default());

    actions
}

fn on_toggle_debug_mode(
    trigger: Trigger<Fired<ToggleDebugMode>>,
    mut commands: Commands,
    debug_modes: Query<Has<CharacterDebugMode>>,
) {
    match debug_modes.get(trigger.target()).unwrap() {
        true => commands
            .entity(trigger.target())
            .remove::<CharacterDebugMode>(),
        false => commands.entity(trigger.target()).insert(CharacterDebugMode),
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
    mut query: Query<(&mut Character, &CharacterMovement)>,
) -> Result {
    let (mut character, movement) = query.get_mut(trigger.target())?;
    character.jump(movement.jump_impulse);
    Ok(())
}

fn move_input(
    mut query: Query<(&mut MoveInput, &Actions<OnFoot>)>,
    camera: Single<&GlobalTransform, With<PlayerCamera>>,
) {
    let mut camera_transform = camera.compute_transform();
    camera_transform.align(Dir3::Y, Dir3::Y, Dir3::NEG_Z, camera_transform.forward());

    for (mut input, actions) in &mut query {
        let axis = actions.action::<Move>().value().as_axis3d();

        input.set(camera_transform.rotation * axis.normalize_or_zero());
    }
}

fn look_input(
    characters: Query<&Actions<OnFoot>>,
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
    targets: Query<&GlobalTransform>,
) -> Result {
    for (mut transform, attached_to) in &mut query {
        let target_transform = targets.get(attached_to.0)?;
        transform.translation = target_transform.translation();
    }

    Ok(())
}
