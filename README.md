🐉 HERE BE DRAGONS 🐉

`slither` privides tools for implementing character movement in the [bevy] game engine, using [avian3d] for physics.
The main function is `move_and_slide` which, you guessed it, moves and slides the character along the world geometry.

```rust
let movment = move_and_slide(position, velocity, ..etc);

position = movement.position;
velocity = movement.velocity;
```

There's also `project_on_floor` which simply projects a vector on a plane, but it does
it in such a way that the resulting vector aligns with the input vector on the horizontal axis
without skewing it down the slope.

It basically allows you to walk up slopes without subtly changing the movement direction, which is anoying.

```rust
let accel = acceleration(velocity, input_direction, ..etc);
velocity += project_on_floor(accel, floor.normal, ..etc);
```

There's also the `CharacterBody` component which automatically calls `move_and_slide` for you in
the `FixedUpdate` schedule.

```rust
commands.spawn((
    CharacterBody::default(),
    InputDirection::default(),
));

#[derive(Component, Default)]
struct InputDirection(Vec3);

fn update(mut query: Query<(&mut CharacterBody, &InputDirection)>, time: Res<Time>) {
    for (mut character, input) in &mut query {
      character.acceleration += input.0 * 100.0 * time.delta_secs();
      character.velocity.y -= 9.8 * time.delta_secs();
    }
}
```

The difference between using `velocity` and `acceleration` for moving the character is that
`acceleration` will be projected onto whatever surface plane the character is touching to avoid
some common issues like being able to walk up steep slopes (steeper than `max_floor_angle`) and
leaving the ground plane when walking down slopes. The rule is generally to use `acceleration`
only for movement input and `velocity` for external forces like gravity.

Feel free to check out the [example](examples/minimal.rs) once I've cleaned it up.

### Todo (now):

- [ ] stair stepping
- [ ] platform velocity
- [ ] 2d support
- [ ] general improvements 

[bevy]: https://github.com/bevyengine/bevy
[avian]: https://github.com/Jondolf/avian
[avian3d]: https://github.com/Jondolf/avian
