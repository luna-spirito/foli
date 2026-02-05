#import bevy_pbr::{
    mesh_view_bindings as view_bindings,
    prepass_utils,
    view_transformations,
}

struct ProjectorData {
    view_proj: mat4x4<f32>,
    intensity: f32,
    _padding: vec3<f32>,
};

@group(#{MATERIAL_BIND_GROUP}) @binding(0)
var<uniform> projector: ProjectorData;
@group(#{MATERIAL_BIND_GROUP}) @binding(1)
var projection_texture: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(2)
var projection_sampler: sampler;
// Shadow Map Bindings
@group(#{MATERIAL_BIND_GROUP}) @binding(3)
var shadow_map: texture_depth_2d;
@group(#{MATERIAL_BIND_GROUP}) @binding(4)
var shadow_sampler: sampler_comparison;

@fragment
fn fragment(
    @builtin(position) frag_coord: vec4<f32>,
) -> @location(0) vec4<f32> {
    // 1. Get the depth of the scene using Bevy prepass
    let depth = prepass_utils::prepass_depth(frag_coord, 0u);

    // Bevy renders proxy mesh even where there is no geometry behind it (skybox).
    // If depth is 0.0 (reversed Z) or 1.0 depending on setup, we might hit sky.
    // In reversed-Z (standard in Bevy), far plane is 0.0.
    if (depth <= 0.0001) {
        discard;
    }

    // 2. Reconstruct World Position from Depth
    // Convert screen coords to NDC: x,y in [-1,1], z is the depth
    let viewport_size = view_bindings::view.viewport.zw;
    let ndc_xy = (frag_coord.xy / viewport_size) * 2.0 - 1.0;
    // Bevy uses +Y up in NDC, but frag_coord.y is top-down, so flip Y
    let ndc = vec3<f32>(ndc_xy.x, -ndc_xy.y, depth);
    let world_pos = view_transformations::position_ndc_to_world(ndc);

    // 3. Project World Position into Projector's Clip Space
    let clip_pos = projector.view_proj * vec4<f32>(world_pos, 1.0);
    // Perspective division to get Projector NDC
    let p_ndc = clip_pos.xyz / clip_pos.w;

    // 4. Convert NDC to UV (0..1)
    // NDC x,y are in [-1, 1]. Y is up in NDC.
    // Texture UVs: (0,0) top-left or bottom-left depending on API.
    // Standard: u = x*0.5 + 0.5, v = -y*0.5 + 0.5 (flip Y)
    let uv = vec2<f32>(p_ndc.x * 0.5 + 0.5, -p_ndc.y * 0.5 + 0.5);

    // 5. Bounds Check (Clipping)
    // Check if point is inside the projector's frustum
    // Also check w > 0 to avoid back-projection
    if (clip_pos.w > 0.0 && p_ndc.x >= -1.0 && p_ndc.x <= 1.0 && p_ndc.y >= -1.0 && p_ndc.y <= 1.0 && p_ndc.z >= 0.0 && p_ndc.z <= 1.0) {

        // 6. Shadow Mapping
        // shadow_map stores depth from projector's perspective (reversed-Z).
        // In reversed-Z: near plane = 1.0, far plane = 0.0
        // p_ndc.z is the depth of the current fragment in projector space.
        // With GreaterEqual comparison and reversed-Z:
        // - Shadow map has the closest (largest) depth value seen from projector
        // - Fragment is lit if its depth >= shadow depth (it's at least as close)
        // - We ADD bias to avoid self-shadowing by making fragment appear slightly closer
        let bias = 0.005; // Bias to prevent shadow acne
        let shadow_vis = textureSampleCompare(
            shadow_map,
            shadow_sampler,
            uv,
            p_ndc.z + bias
        );

        // If shadow_vis is 0.0, it's shadowed. 1.0 means visible.
        if (shadow_vis < 0.1) {
             discard;
             // return vec4<f32>(0.0, 0.0, 0.0, 0.5); // Debug shadow
        }

        // 7. Sample Projection Texture
        let color = textureSample(projection_texture, projection_sampler, uv);

        // Additive Blending logic
        if (color.a < 0.01) {
            discard;
        }

        // Final Output
        return vec4<f32>(color.rgb * projector.intensity * color.a, color.a);
    }

    discard;
}
