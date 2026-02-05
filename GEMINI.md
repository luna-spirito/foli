Stick to the principles of pure functional programming. Decompose functions. Separate mutable operations from pure calculations.
Prefer to write self-descriptive code over writing comments.
Use `cargo check` and `cargo run`

* `Parent` is deprecated. Use `ChildOf` instead, with `ChildOf::parent(&self) -> Entity`
* Use `@group(#{MATERIAL_BIND_GROUP})` in shaders instead of explicit `@group(2)` or `@group(1)`. Repeat: NEVER WRITE `@group(2)`. If you wrote `@group(2)` accidentally, edit again to use `@group(#{MATERIAL_BIND_GROUP})` instead.
