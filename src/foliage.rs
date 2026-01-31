use bevy::{
    pbr::{ExtendedMaterial, MaterialExtension},
    prelude::*,
    render::render_resource::{AsBindGroup, ShaderType},
    shader::ShaderRef,
};

pub struct FoliagePlugin;

impl Plugin for FoliagePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<FoliageMaterial>::default())
            .add_systems(Update, apply_foliage_materials);
    }
}

#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct FoliageExtension {
    #[uniform(100)]
    pub data: FoliageData,
}

#[derive(ShaderType, Clone, Debug, Reflect, Default)]
pub struct FoliageData {
    pub wind_speed: f32,
    pub wind_amplitude: f32,
}

impl MaterialExtension for FoliageExtension {
    fn vertex_shader() -> ShaderRef {
        "shaders/foliage.wgsl".into()
    }

    fn fragment_shader() -> ShaderRef {
        "shaders/foliage.wgsl".into()
    }
}

pub type FoliageMaterial = ExtendedMaterial<StandardMaterial, FoliageExtension>;

fn apply_foliage_materials(
    mut commands: Commands,
    mut foliage_materials: ResMut<Assets<FoliageMaterial>>,
    std_materials: Res<Assets<StandardMaterial>>,
    query: Query<(Entity, &MeshMaterial3d<StandardMaterial>, &Name), Added<Mesh3d>>,
) {
    for (entity, mat_handle, name) in &query {
        if name.contains("Leaf") || name.contains("Foliage") {
            if let Some(std_mat) = std_materials.get(mat_handle) {
                let mut new_mat = std_mat.clone();
                new_mat.cull_mode = None;
                new_mat.double_sided = true; // Essential for flat planes
                new_mat.alpha_mode = AlphaMode::Blend;

                // Light transmission makes foliage look "alive" when lit from behind
                new_mat.diffuse_transmission = 0.3;

                let foliage_mat = FoliageMaterial {
                    base: new_mat,
                    extension: FoliageExtension {
                        data: FoliageData {
                            wind_speed: 1.5,
                            wind_amplitude: 0.1,
                        },
                    },
                };

                commands
                    .entity(entity)
                    .remove::<MeshMaterial3d<StandardMaterial>>();
                commands
                    .entity(entity)
                    .insert(MeshMaterial3d(foliage_materials.add(foliage_mat)));
            }
        }
    }
}
