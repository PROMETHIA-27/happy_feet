🐉 HERE BE DRAGONS 🐉

`slither` privides tools for implementing character movement in the bevy engine, using avian3d for physics.
The main function is `move_and_slide` which, you guessed it, moves and slides the character along the world geometry.

```rust
let movment = slither::move_and_slide(position, velocity, ..etc);

position = movement.position;
velocity = movement.velocity;
```

There's also `project_on_floor` which simply projects a vector on a plane, but it does
it in such a way that the resulting vector aligns with the input vector on the horizontal axis
without skewing it down the slope.

It basically allows you to walk up slopes without subtly changing the movement direction, which is anoying.

```rust
let accel = acceleration(velocity, input_direction, ..etc); // Simple quake style acceleration, not part of the library.
velocity += project_on_floor(accel, floor.normal, ..etc);
```

That's it! 

Feel free to check out the [example](examples/minimal.rs) once I've cleaned it up.
