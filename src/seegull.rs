use std::f32::consts::{PI, TAU};

use avian3d::prelude::*;
use bevy::prelude::*;

pub struct SeegullPlugin;

impl Plugin for SeegullPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (follow, orbit, look_at, spring_arm, update_transform).chain(),
        );
    }
}

#[derive(Component, Default)]
pub struct ViewTransform {
    pub origin: Vec3,
    pub offset: Vec3,
    pub rotation: Quat,
}

#[derive(Component)]
pub struct Follow {
    pub entity: Entity,
    pub offset: Vec3,
    pub easing: f32,
}

#[derive(Default)]
pub enum OrbitMode {
    Translate,
    Rotate,
    #[default]
    All,
}

#[derive(Component)]
pub struct Orbit {
    pub mode: OrbitMode,
    pub offset: Vec3,
    pub easing: f32,
    pub rotation: Rotation,
}

impl Default for Orbit {
    fn default() -> Self {
        Self {
            rotation: Rotation::IDENTITY,
            offset: Vec3::ZERO,
            easing: 0.0,
            mode: OrbitMode::All,
        }
    }
}

#[derive(Component, Debug, Clone, Copy)]
pub struct LookAt {
    pub entity: Entity,
    pub offset: Vec3,
    pub easing: f32,
}

#[derive(Component)]
pub struct SpringArm {
    pub filter: SpatialQueryFilter,
    pub easing: f32,
    pub radius: f32,
    pub distance: f32,
}

fn follow(
    mut query: Query<(&mut ViewTransform, &Follow)>,
    other: Query<&Transform, Without<Follow>>,
    time: Res<Time>,
) {
    for (
        mut view,
        &Follow {
            entity,
            offset,
            easing,
        },
    ) in &mut query
    {
        let Ok(target) = other.get(entity) else {
            continue;
        };
        // view.origin = other_transform.translation + offset;
        let target = target.translation + offset;
        match easing > 0.0 {
            true => {
                view.origin
                    .smooth_nudge(&target, 1.0 / easing, time.delta_secs());
            }
            false => {
                view.origin = target;
            }
        };
    }
}

fn look_at(
    mut query: Query<(&mut ViewTransform, &LookAt)>,
    other: Query<&Transform>,
    time: Res<Time>,
) {
    for (mut view, look) in &mut query {
        let Ok(target) = other.get(look.entity) else {
            continue;
        };

        let Ok(direction) = Dir3::new(target.translation - view.origin - view.offset) else {
            continue;
        };

        let target = Transform::default().looking_to(direction, Dir3::Y).rotation;

        match look.easing > 0.0 {
            true => {
                let decay_rate = 1.0 / look.easing;
                view.rotation = view.rotation.slerp(target, decay_rate * time.delta_secs());
            }
            false => view.rotation = target,
        }
    }
}

fn orbit(mut query: Query<(&mut ViewTransform, &Orbit)>, time: Res<Time>) {
    for (mut view, orbit) in &mut query {
        let target_offset = orbit.rotation.to_quat() * orbit.offset;

        if matches!(orbit.mode, OrbitMode::Rotate | OrbitMode::All) {
            match orbit.easing > 0.0 {
                true => {
                    let decay_rate = 1.0 / orbit.easing;

                    view.rotation = view
                        .rotation
                        .slerp(orbit.rotation.to_quat(), decay_rate * time.delta_secs());
                }
                false => {
                    view.rotation = orbit.rotation.to_quat();
                }
            }
        }

        let Ok((direction, distance)) = Dir3::new_and_length(target_offset) else {
            continue;
        };

        if matches!(orbit.mode, OrbitMode::Translate | OrbitMode::All) {
            match orbit.easing > 0.0 {
                true => {
                    let decay_rate = 1.0 / orbit.easing;

                    let Ok(old_direction) = Dir3::new(view.offset) else {
                        view.offset = target_offset;
                        continue;
                    };

                    let new_direction =
                        old_direction.slerp(direction, decay_rate * time.delta_secs());
                    view.offset = new_direction * distance;
                }

                false => {
                    view.offset = target_offset;
                }
            }
        }
    }
}

fn spring_arm(
    spatial: SpatialQuery,
    mut query: Query<(&mut ViewTransform, &mut SpringArm)>,
    time: Res<Time>,
) {
    for (mut view, mut spring_arm) in &mut query {
        let Ok((direction, distance)) = Dir3::new_and_length(view.offset) else {
            continue;
        };

        // TODO: this should probably be a shape cast.
        match spatial.cast_ray(
            view.origin,
            direction,
            distance + spring_arm.radius,
            false,
            &spring_arm.filter,
        ) {
            Some(hit) => spring_arm.distance = hit.distance - spring_arm.radius,
            None => match spring_arm.easing > 0.0 {
                true => {
                    let decay_rate = 1.0 / spring_arm.easing;
                    spring_arm
                        .distance
                        .smooth_nudge(&distance, decay_rate, time.delta_secs())
                }
                false => spring_arm.distance = distance,
            },
        }

        view.offset = direction * spring_arm.distance;
    }
}

fn update_transform(mut query: Query<(&mut Transform, &ViewTransform)>) {
    for (mut transform, view) in &mut query {
        transform.translation = view.origin + view.offset;
        transform.rotation = view.rotation;
    }
}

#[derive(Default, Debug, PartialEq, Clone, Copy)]
#[repr(C)]
pub struct Euler {
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

impl Euler {
    pub const fn new(pitch: f32, yaw: f32, roll: f32) -> Self {
        Self { pitch, yaw, roll }
    }

    pub const fn from_vec(vec: Vec3) -> Self {
        Self::new(vec.x, vec.y, vec.z)
    }

    pub const fn to_vec(self) -> Vec3 {
        Vec3::new(self.pitch, self.yaw, self.roll)
    }

    pub const fn from_angles(order: EulerRot, a: f32, b: f32, c: f32) -> Self {
        let (pitch, yaw, roll) = match order {
            EulerRot::ZYX => (c, b, a),
            EulerRot::ZXY => (b, c, a),
            EulerRot::YXZ => (b, a, c),
            EulerRot::YZX => (c, a, c),
            EulerRot::XYZ => (a, b, c),
            EulerRot::XZY => (a, c, b),
            _ => unimplemented!(),
        };
        Self { pitch, yaw, roll }
    }

    pub const fn to_angles(self, order: EulerRot) -> (f32, f32, f32) {
        let Self { pitch, yaw, roll } = self;
        match order {
            EulerRot::ZYX => (roll, yaw, pitch),
            EulerRot::ZXY => (roll, pitch, yaw),
            EulerRot::YXZ => (yaw, pitch, roll),
            EulerRot::YZX => (yaw, roll, pitch),
            EulerRot::XYZ => (pitch, yaw, roll),
            EulerRot::XZY => (pitch, roll, yaw),
            _ => unimplemented!(),
        }
    }

    pub fn set_pitch(&mut self, pitch: f32) {
        self.pitch = match pitch > PI || pitch < -PI {
            true => pitch % TAU - pitch.signum() * TAU,
            false => pitch,
        };
    }

    pub fn set_yaw(&mut self, yaw: f32) {
        self.yaw = match yaw > PI || yaw < -PI {
            true => yaw % TAU - yaw.signum() * TAU,
            false => yaw,
        };
    }

    pub fn set_roll(&mut self, roll: f32) {
        self.roll = match roll > PI || roll < -PI {
            true => roll % TAU - roll.signum() * TAU,
            false => roll,
        };
    }

    pub fn add_pitch(&mut self, pitch: f32) {
        self.set_pitch(self.pitch + pitch);
    }

    pub fn add_yaw(&mut self, yaw: f32) {
        self.set_yaw(self.yaw + yaw);
    }

    pub fn add_roll(&mut self, roll: f32) {
        self.set_roll(self.roll + roll);
    }

    pub fn to_quat(self, order: EulerRot) -> Quat {
        let (a, b, c) = self.to_angles(order);
        Quat::from_euler(order, a, b, c)
    }

    pub fn from_quat(order: EulerRot, quat: Quat) -> Self {
        let (a, b, c) = quat.to_euler(order);
        Self::from_angles(order, a, b, c)
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Rotation {
    Euler { value: Euler, order: EulerRot },
    Quaternion { value: Quat, order: EulerRot },
}

impl Default for Rotation {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Rotation {
    pub const IDENTITY: Self = Self::Quaternion {
        value: Quat::IDENTITY,
        order: EulerRot::YXZ,
    };

    pub fn set_pitch(&mut self, pitch: f32) {
        match self {
            Rotation::Euler { value, .. } => value.set_pitch(pitch),
            Rotation::Quaternion { value, order } => {
                let mut euler = Euler::from_quat(*order, *value);
                euler.set_pitch(pitch);
                *value = euler.to_quat(*order);
            }
        }
    }

    pub fn set_yaw(&mut self, yaw: f32) {
        match self {
            Rotation::Euler { value, .. } => value.set_yaw(yaw),
            Rotation::Quaternion { value, order } => {
                let mut euler = Euler::from_quat(*order, *value);
                euler.set_yaw(yaw);
                *value = euler.to_quat(*order);
            }
        }
    }

    pub fn set_roll(&mut self, roll: f32) {
        match self {
            Rotation::Euler { value, .. } => value.set_roll(roll),
            Rotation::Quaternion { value, order } => {
                let mut euler = Euler::from_quat(*order, *value);
                euler.set_roll(roll);
                *value = euler.to_quat(*order);
            }
        }
    }

    pub fn euler_mut(&mut self) -> &mut Euler {
        if let Self::Quaternion { value, order } = *self {
            *self = Self::Euler {
                value: Euler::from_quat(order, value),
                order,
            };
        }

        match self {
            Rotation::Euler { value, .. } => value,
            Rotation::Quaternion { .. } => unreachable!(),
        }
    }

    pub fn quat_mut(&mut self) -> &mut Quat {
        if let Self::Euler { value, order } = *self {
            *self = Self::Quaternion {
                value: value.to_quat(order),
                order,
            };
        }

        match self {
            Rotation::Quaternion { value, .. } => value,
            Rotation::Euler { .. } => unreachable!(),
        }
    }

    pub fn order_mut(&mut self) -> &mut EulerRot {
        match self {
            Rotation::Euler { order, .. } => order,
            Rotation::Quaternion { order, .. } => order,
        }
    }

    pub fn to_quat(self) -> Quat {
        match self {
            Self::Quaternion { value, .. } => value,
            Self::Euler { value, order } => value.to_quat(order),
        }
    }

    pub fn to_euler(self) -> Euler {
        match self {
            Self::Quaternion { value, order } => Euler::from_quat(order, value),
            Self::Euler { value, .. } => value,
        }
    }
}
