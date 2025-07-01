Work in progress kinematic character controller for the bevy game engine.

> [!WARNING]
> Here be penguins! 🐧

---

🧊 Set up the app

```toml
# Cargo.toml
[dependencies]
avian3d = "0.3.1"
bevy = "0.16.1"
happy_feet = { git = "https://github.com/atornity/happy_feet.git" }
```

```rust
// main.rs
use bevy::prelude::*;
use avian3d::prelude::*;
use happy_feet::prelude::*;

fn main() -> AppExit {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            CharacterPlugins::default(), // This is us!
        ))
        .run()
}
```

🐧 Spawn the character

```rust
fn setup(mut commands: Commands) {
    commands.spawn((
        // We technically only need the character component, 
        // the following components are optional
        Character,
        // Enables automatic acceleration of the character when updating the MoveInput component
        // If you want custom movement, add MoveInput instead and use it to update KinematicVelocity directly
        CharacterMovement {
            target_speed: 8.0,
            acceleration: 60.0,
        },
        // Enables gravity for the character, uses the Gravity resource by default
        CharacterGravity::default(),
        // Enables friction while moving on the ground
        GroundFriction(40.0),
        // Enables drag
        CharacterDrag(0.01),
    ));
}
```

🎿 Move the character

```rust
fn character_input(
    key: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&mut MoveInput, &mut KinematicVelocity, &mut Grounding)>,
) {
    let move_direction = get_move_direction(&key); // This is left as an exercise for the reader (^:

    for (mut move_input, mut velocity, mut grounding) in &mut query {
        move_input.value = move_direction;

        if grounding.is_grounded() && key.just_pressed(KeyCode::Space) {
            velocity.y = 6.0;
            grounding.detach(); // Detach from the ground to avoid snapping back to it during character update
        }
    }
}
```

That's it! Feel free to check out the [examples](examples/minimal.rs) as well.
