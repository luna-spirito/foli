use bevy::{
    asset::RenderAssetUsages,
    core_pipeline::core_3d::graph::{Core3d, Node3d},
    ecs::{query::QueryItem, system::lifetimeless::Read},
    image::{ImageCompareFunction, ImageSampler, ImageSamplerDescriptor},
    prelude::*,
    render::{
        RenderApp,
        camera::ExtractedCamera,
        extract_component::ExtractComponent,
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        render_asset::RenderAssets,
        render_graph::{
            NodeRunError, RenderGraphContext, RenderGraphExt, RenderLabel, ViewNode, ViewNodeRunner,
        },
        render_resource::{
            Extent3d, Origin3d, TexelCopyTextureInfo, TextureAspect, TextureDescriptor,
            TextureDimension, TextureFormat, TextureUsages,
        },
        renderer::RenderContext,
        texture::GpuImage,
        view::ViewDepthTexture,
    },
};

pub const SHADOW_MAP_SIZE: u32 = 1024;
#[derive(Component, Clone, ExtractComponent)]
pub struct FoliViewCameraTag;

/// Resource to hold the shadow map texture handle
#[derive(Resource, Clone, ExtractResource)]
pub struct FoliViewMap(pub Handle<Image>);

impl FromWorld for FoliViewMap {
    fn from_world(world: &mut World) -> Self {
        let mut images = world.resource_mut::<Assets<Image>>();
        let size = Extent3d {
            width: SHADOW_MAP_SIZE,
            height: SHADOW_MAP_SIZE,
            depth_or_array_layers: 1,
        };

        // Create GPU-only depth texture (NO initial data - depth formats forbid write_texture)
        let image = Image {
            data: None, // No CPU-side data - depth textures are GPU-only
            texture_descriptor: TextureDescriptor {
                label: Some("Projector Shadow Map"),
                size,
                mip_level_count: 1,
                sample_count: 1,
                dimension: TextureDimension::D2,
                format: TextureFormat::Depth32Float,
                usage: TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST,
                view_formats: &[],
            },
            sampler: ImageSampler::Descriptor(ImageSamplerDescriptor {
                label: Some("Projector Shadow Map Sampler".into()),
                // GreaterEqual for reversed-Z: fragment is lit if its depth >= stored shadow depth
                compare: Some(ImageCompareFunction::GreaterEqual),
                ..default()
            }),
            asset_usage: RenderAssetUsages::RENDER_WORLD,
            ..default()
        };

        FoliViewMap(images.add(image))
    }
}

/// Render Graph Label for the shadow copy pass
#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
struct CopyShadowMapPass;

/// The Render Graph Node that copies the depth texture
#[derive(Default)]
struct CopyShadowMapNode;

impl ViewNode for CopyShadowMapNode {
    type ViewQuery = (
        Read<ExtractedCamera>,
        Read<ViewDepthTexture>,
        Option<Read<FoliViewCameraTag>>,
    );

    fn run<'w>(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext<'w>,
        (_camera, depth_texture, shadow_tag): QueryItem<'w, '_, Self::ViewQuery>,
        world: &'w World,
    ) -> Result<(), NodeRunError> {
        // Only run for the Shadow Camera (identified by tag)
        if shadow_tag.is_none() {
            return Ok(());
        }

        let shadow_map_res = world.resource::<FoliViewMap>();
        let gpu_images = world.resource::<RenderAssets<GpuImage>>();

        if let Some(gpu_image) = gpu_images.get(&shadow_map_res.0) {
            let encoder = render_context.command_encoder();

            encoder.copy_texture_to_texture(
                TexelCopyTextureInfo {
                    texture: &depth_texture.texture,
                    mip_level: 0,
                    origin: Origin3d::default(),
                    aspect: TextureAspect::DepthOnly,
                },
                TexelCopyTextureInfo {
                    texture: &gpu_image.texture,
                    mip_level: 0,
                    origin: Origin3d::default(),
                    aspect: TextureAspect::DepthOnly,
                },
                Extent3d {
                    width: SHADOW_MAP_SIZE,
                    height: SHADOW_MAP_SIZE,
                    depth_or_array_layers: 1,
                },
            );
        }

        Ok(())
    }
}

pub struct FoliViewPlugin;

impl Plugin for FoliViewPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(ExtractResourcePlugin::<FoliViewMap>::default()) // ???
            .init_resource::<FoliViewMap>();

        // Render Graph Setup
        if let Some(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app.add_render_graph_node::<ViewNodeRunner<CopyShadowMapNode>>(
                Core3d,
                CopyShadowMapPass,
            );

            render_app.add_render_graph_edges(
                Core3d,
                (
                    Node3d::EndPrepasses,
                    CopyShadowMapPass,
                    Node3d::MainOpaquePass,
                ),
            );
        }
    }
}
