use bevy::prelude::*;

fn main() {
    App::new()
        // DefaultPlugins adds the window, input handling, and the renderer
        .add_plugins(DefaultPlugins)
        // Startup systems run once at the beginning
        .add_systems(Startup, setup)
        // Update systems run every frame
        .add_systems(Update, rotate_sprite)
        .run();
}

#[derive(Component)]
struct Player;

fn setup(mut commands: Commands) {
    // We must spawn a camera to see anything
    commands.spawn(Camera2d);

    // Spawn a green square
    commands.spawn((
        Sprite {
            color: Color::srgb(0.0, 1.0, 0.0),
            custom_size: Some(Vec2::new(100.0, 100.0)),
            ..default()
        },
        Player,
    ));
}

fn rotate_sprite(time: Res<Time>, mut query: Query<&mut Transform, With<Player>>) {
    for mut transform in &mut query {
        // Rotate the square based on time passed
        transform.rotate_z(time.delta_secs());
    }
}
