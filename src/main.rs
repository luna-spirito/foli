mod movement;
mod render;

use std::f32;

use crate::movement::{BipedalCfg, apply_ik, debug_show_axis, locomotion, setup_bipedal};
use bevy::{
    core_pipeline::prepass::{DepthPrepass, NormalPrepass},
    gltf::{GltfPlugin, convert_coordinates::GltfConvertCoordinates},
    prelude::*,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(GltfPlugin {
            convert_coordinates: GltfConvertCoordinates {
                rotate_scene_entity: true,
                rotate_meshes: false,
            },
            ..default()
        }))
        .add_plugins(render::foliage::FoliagePlugin)
        .add_plugins(render::projector::ProjectorPlugin)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                setup_bipedal,
                // Chain the input service with the locomotion brain
                wasd_input.pipe(locomotion),
                apply_ik,
                debug_show_axis,
            ),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut _meshes: ResMut<Assets<Mesh>>,
    mut _materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(7.0, 5.0, 9.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
        DepthPrepass,
        NormalPrepass,
    ));

    // Light
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(-2.0, 5.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Model
    commands.spawn((
        SceneRoot(asset_server.load("models/bastard.glb#Scene0")),
        Transform::from_xyz(0.0, 0.0, 0.0),
        BipedalCfg {
            speed: 1.5,
            step_duration: 0.25,
            step_height: 0.2,
            ankle_height: 0.07, //0.05,
                                // torso_off_min: -0.1,
                                // torso_off_sway: 0.05,
        },
        Player, // Mark as player controlled
    ));

    commands.spawn((
        SceneRoot(asset_server.load("models/trees/1.gltf#Scene0")),
        Transform::from_xyz(0.0, 0.0, 0.0)
            .with_rotation(Quat::from_rotation_y(f32::consts::PI / 1.5)),
    ));
}

#[derive(Component)]
struct Player;

/// Service that reads WASD input and produces a target position for the Player
fn wasd_input(
    keys: Res<ButtonInput<KeyCode>>,
    player_query: Query<&Transform, With<Player>>,
) -> Option<Vec3> {
    let Some(transform) = player_query.iter().next() else {
        return None;
    };

    let mut dir = Vec3::ZERO;
    // Basic WSAD relative to world for now (could be relative to camera later)
    if keys.pressed(KeyCode::KeyW) {
        dir.z -= 1.0;
    }
    if keys.pressed(KeyCode::KeyS) {
        dir.z += 1.0;
    }
    if keys.pressed(KeyCode::KeyA) {
        dir.x -= 1.0;
    }
    if keys.pressed(KeyCode::KeyD) {
        dir.x += 1.0;
    }

    if dir == Vec3::ZERO {
        return None;
    }

    // Target is slightly ahead in the desired direction
    // We normalize to ensure consistent speed regardless of diagonal input
    Some(transform.translation + dir.normalize() * 1.0)
}

// #[derive(Component)]
// struct Walker {
//     speed: f32,
// }
