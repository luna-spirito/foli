use bevy::{
    camera::{CameraProjection, RenderTarget, visibility::RenderLayers},
    core_pipeline::prepass::DepthPrepass,
    light::NotShadowCaster,
    mesh::MeshVertexBufferLayoutRef,
    pbr::{Material, MaterialPipeline, MaterialPipelineKey},
    prelude::*,
    render::render_resource::{
        AsBindGroup, CompareFunction, Extent3d, Face, RenderPipelineDescriptor, ShaderType,
        SpecializedMeshPipelineError, TextureDescriptor, TextureDimension, TextureFormat,
        TextureUsages,
    },
    shader::ShaderRef,
};

use crate::render::foli_view::{self, FoliViewCameraTag, FoliViewMap};

/// The render layer for holographic objects
pub const HOLOGRAM_LAYER: RenderLayers = RenderLayers::layer(10);

pub struct ProjectorPlugin;

impl Plugin for ProjectorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<ProjectorDecalMaterial>::default())
            .add_systems(Startup, setup_projector_infrastructure)
            .add_systems(
                Update,
                (
                    update_projector_uniforms,
                    draw_projector_debug,
                    animate_hologram,
                    attach_projector_to_player,
                ),
            );
    }
}

#[derive(ShaderType, Clone, Debug, Reflect, Default)]
pub struct ProjectorUniform {
    pub view_proj: Mat4,
    pub intensity: f32,
    pub _padding: Vec3,
}

#[derive(Asset, AsBindGroup, Reflect, Debug, Clone)]
pub struct ProjectorDecalMaterial {
    #[uniform(0)]
    pub data: ProjectorUniform,
    #[texture(1)]
    #[sampler(2)]
    pub texture: Handle<Image>,
    #[texture(3, sample_type = "depth")]
    #[sampler(4, sampler_type = "comparison")]
    pub shadow_map: Handle<Image>,
}

impl Material for ProjectorDecalMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/projector.wgsl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Blend
    }

    fn specialize(
        _pipeline: &MaterialPipeline,
        descriptor: &mut RenderPipelineDescriptor,
        _layout: &MeshVertexBufferLayoutRef,
        _key: MaterialPipelineKey<Self>,
    ) -> Result<(), SpecializedMeshPipelineError> {
        descriptor.primitive.cull_mode = Some(Face::Front);
        // Disable depth testing.
        // Since we're only showing back faces, they are easily beyond the scene boundary. So we disable depth testing.
        if let Some(depth_stencil) = &mut descriptor.depth_stencil {
            depth_stencil.depth_compare = CompareFunction::Always;
            depth_stencil.depth_write_enabled = false;
        }
        Ok(())
    }
}

#[derive(Component)]
pub struct ARProjector {
    pub intensity: f32,
}

#[derive(Component)]
struct HologramTag;

#[derive(Component)]
struct ProjectorCameraTag;

#[derive(Component)]
struct DecalVolumeTag;

#[derive(Component)]
struct AttachedToHead;

fn setup_projector_infrastructure(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut decal_materials: ResMut<Assets<ProjectorDecalMaterial>>,
    projector_shadow_map: Res<FoliViewMap>,
) {
    // 1. Create the RTT texture
    let size = Extent3d {
        width: 1024,
        height: 1024,
        ..default()
    };
    let mut image = Image {
        texture_descriptor: TextureDescriptor {
            label: Some("Projector RTT Texture"),
            size,
            dimension: TextureDimension::D2,
            format: TextureFormat::Bgra8UnormSrgb,
            mip_level_count: 1,
            sample_count: 1,
            usage: TextureUsages::TEXTURE_BINDING
                | TextureUsages::COPY_DST
                | TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        },
        ..default()
    };
    image.resize(size);
    let image_handle = images.add(image);

    // 2. Spawn the Projector Camera (Internal)
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: 90.0f32.to_radians(),
            near: 0.1,
            far: 20.0,
            ..default()
        }),
        Camera {
            clear_color: ClearColorConfig::Custom(Color::NONE),
            ..default()
        },
        RenderTarget::Image(image_handle.clone().into()),
        Msaa::Off,
        Transform::default(),
        HOLOGRAM_LAYER,
        ARProjector { intensity: 10.0 },
        ProjectorCameraTag,
    ));

    // 3. Spawn the Shadow Camera (Depth Only)
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: 90.0f32.to_radians(),
            near: 0.1,
            far: 20.0,
            ..default()
        }),
        Camera {
            order: -1, // Render first
            ..default()
        },
        // No RenderTarget (Manual size for Depth Texture)
        RenderTarget::None {
            size: UVec2::splat(foli_view::SHADOW_MAP_SIZE),
        },
        Msaa::Off,
        DepthPrepass,
        Transform::default(),
        // Shadow camera sees World (0) but NOT Holograms (10)
        RenderLayers::default().with(0).without(10),
        foli_view::FoliViewCameraTag,
    ));

    // 4. Create the Projector Material
    let projector_material = decal_materials.add(ProjectorDecalMaterial {
        data: ProjectorUniform::default(),
        texture: image_handle.clone(),
        shadow_map: projector_shadow_map.0.clone(),
    });

    // 5. Create the Decal Volume
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(Vec3::splat(1.0)))),
        MeshMaterial3d(projector_material),
        Transform::default(),
        NotShadowCaster,
        DecalVolumeTag,
    ));

    // 4. Setup some standard world geometry
    // Floor
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(10.0, 10.0))),
        MeshMaterial3d(materials.add(StandardMaterial::from(Color::srgb(0.5, 0.5, 0.5)))),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // An obstacle (Wall)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(2.0, 1.0, 0.2))),
        MeshMaterial3d(materials.add(StandardMaterial::from(Color::srgb(0.6, 0.4, 0.4)))),
        Transform::from_xyz(0.0, 0.5, 1.0),
    ));

    // Cubes on the floor (debris)
    for x in [-2.0, 1.5] {
        for z in [-1.0, 2.0] {
            commands.spawn((
                Mesh3d(meshes.add(Cuboid::from_size(Vec3::splat(0.4)))),
                MeshMaterial3d(materials.add(StandardMaterial::from(Color::srgb(0.4, 0.4, 0.6)))),
                Transform::from_xyz(x, 0.2, z),
            ));
        }
    }

    // 5. Setup the Hologram Cube (Only visible to the projector)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(Vec3::splat(1.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::linear_rgba(0.0, 1.0, 0.5, 0.5),
            emissive: LinearRgba::new(0.0, 2.0, 1.0, 1.0),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, 1.0, 0.0),
        HOLOGRAM_LAYER,
        HologramTag,
    ));

    for i in -5..5 {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(10.0, 0.01, 0.01))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::linear_rgba(0.0, 1.0, 0.5, 0.5),
                emissive: LinearRgba::new(0.0, 2.0, 1.0, 1.0),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, i as f32 * 1.0),
            HOLOGRAM_LAYER,
        ));
    }
    for j in -5..5 {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.01, 0.01, 10.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::linear_rgba(0.0, 1.0, 0.5, 0.5),
                emissive: LinearRgba::new(0.0, 2.0, 1.0, 1.0),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_xyz(j as f32 * 1.0, 0.0, 0.0),
            HOLOGRAM_LAYER,
        ));
    }
}

fn attach_projector_to_player(
    mut commands: Commands,
    player_head_query: Query<
        (Entity, &Name),
        (Without<AttachedToHead>, Without<ProjectorCameraTag>),
    >,
    projector_camera_query: Query<Entity, With<ProjectorCameraTag>>,
    shadow_camera_query: Query<Entity, With<FoliViewCameraTag>>,
    decal_volume_query: Query<Entity, With<DecalVolumeTag>>,
) {
    let Some(projector_cam) = projector_camera_query.iter().next() else {
        return;
    };
    let Some(shadow_cam) = shadow_camera_query.iter().next() else {
        return;
    };
    let Some(decal_vol) = decal_volume_query.iter().next() else {
        return;
    };

    for (head_entity, name) in &player_head_query {
        if name.as_str() == "mixamorig:Head" {
            println!("Attaching Projector to Head: {:?}", head_entity);

            commands.entity(head_entity).add_child(projector_cam);
            commands.entity(head_entity).add_child(shadow_cam);
            commands.entity(head_entity).insert(AttachedToHead);

            let offset = Transform::from_xyz(0.0, 0.1, -0.2).looking_to(Vec3::NEG_Z, Vec3::Y);

            commands
                .entity(projector_cam)
                .insert((offset, AttachedToHead));

            commands.entity(shadow_cam).insert((offset, AttachedToHead));

            commands.entity(head_entity).add_child(decal_vol);
            commands.entity(decal_vol).insert(Transform::default());
        }
    }
}

fn animate_hologram(time: Res<Time>, mut query: Query<&mut Transform, With<HologramTag>>) {
    let t = time.elapsed_secs();
    for mut transform in query.iter_mut() {
        transform.translation.y = 1.0 + (t * 2.0).sin() * 0.5;
        transform.rotation = Quat::from_rotation_y(t) * Quat::from_rotation_x(t * 0.5);
    }
}

fn update_projector_uniforms(
    projector_query: Query<(&GlobalTransform, &ARProjector, &Projection)>,
    mut decal_query: Query<
        &mut Transform,
        (
            With<MeshMaterial3d<ProjectorDecalMaterial>>,
            Without<ARProjector>,
        ),
    >,
    mut materials: ResMut<Assets<ProjectorDecalMaterial>>,
) {
    for (transform, projector, projection) in projector_query.iter() {
        let (proj, near, far) = match projection {
            Projection::Perspective(p) => (p.get_clip_from_view(), p.near, p.far),
            Projection::Orthographic(p) => (p.get_clip_from_view(), p.near, p.far),
            _ => (Mat4::IDENTITY, 0.1, 1000.0), // Fallback for custom or unknown projections
        };

        let view = transform.to_matrix().inverse();
        let view_proj = proj * view;

        for mut local_transform in decal_query.iter_mut() {
            let dist_to_center = (far + near) * 0.5;
            local_transform.translation = Vec3::NEG_Z * dist_to_center;
            local_transform.scale = Vec3::new(far * 2.0, far * 2.0, far * 1.1);
        }

        for (_, material) in materials.iter_mut() {
            material.data.view_proj = view_proj;
            material.data.intensity = projector.intensity;
        }
    }
}

fn draw_projector_debug(mut gizmos: Gizmos, query: Query<(&GlobalTransform, &ARProjector)>) {
    for (gtr, _) in query.iter() {
        gizmos.sphere(gtr.translation(), 0.1, Color::WHITE);
    }
    // return;
    // for (gtr, _projector, projection) in query.iter() {
    //     let (fov, _near, _far) = match projection {
    //         Projection::Perspective(p) => (p.fov, p.near, p.far),
    //         _ => (45.0f32.to_radians(), 0.1, 20.0),
    //     };

    //     let pos = gtr.translation();
    //     let transform = gtr.compute_transform();

    //     let forward = transform.rotation * Vec3::NEG_Z;
    //     let up = transform.rotation * Vec3::Y;
    //     let right = transform.rotation * Vec3::X;

    //     gizmos.sphere(pos, 0.1, Color::WHITE);

    //     let dist = 2.0;
    //     let spread = (fov / 2.0).tan() * dist;

    //     let corners = [
    //         pos + forward * dist + up * spread + right * spread,
    //         pos + forward * dist + up * spread - right * spread,
    //         pos + forward * dist - up * spread - right * spread,
    //         pos + forward * dist - up * spread + right * spread,
    //     ];

    //     for corner in corners {
    //         gizmos.line(pos, corner, Color::srgb(0.0, 1.0, 0.0));
    //     }
    //     gizmos.line(corners[0], corners[1], Color::srgb(0.0, 1.0, 0.0));
    //     gizmos.line(corners[1], corners[2], Color::srgb(0.0, 1.0, 0.0));
    //     gizmos.line(corners[2], corners[3], Color::srgb(0.0, 1.0, 0.0));
    //     gizmos.line(corners[3], corners[0], Color::srgb(0.0, 1.0, 0.0));
    // }
}
