// Foliage shader with wind animation
// Extends StandardMaterial with vertex displacement for natural foliage motion

#import bevy_pbr::{
    mesh_bindings::mesh,
    mesh_functions,
    view_transformations::position_world_to_clip,
}

#import bevy_pbr::mesh_view_bindings::globals

#ifdef PREPASS_PIPELINE
#import bevy_pbr::prepass_io::{Vertex, VertexOutput, FragmentOutput}
#import bevy_pbr::pbr_prepass_functions::prepass_alpha_discard
#else
#import bevy_pbr::forward_io::{Vertex, VertexOutput, FragmentOutput}
#import bevy_pbr::pbr_fragment::pbr_input_from_standard_material
#import bevy_pbr::pbr_functions::{alpha_discard, apply_pbr_lighting, main_pass_post_lighting_processing}
#endif

struct FoliageData {
    wind_speed: f32,
    wind_amplitude: f32,
    wind_flutter: f32,
    wind_gustiness: f32,
    time: f32,
    serration: f32,
    irregularity: f32,
    //holes: f32,
    edge_drying: f32,
}

@group(#{MATERIAL_BIND_GROUP}) @binding(100)
var<uniform> foliage_data: FoliageData;

fn hash13(p: vec3<f32>) -> f32 {
    var p3 = fract(p * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

fn wind_displacement(world_pos: vec3<f32>, local_pos: vec3<f32>) -> vec3<f32> {
    let speed = foliage_data.wind_speed;
    let amp = foliage_data.wind_amplitude;
    let time = foliage_data.time;

    // 1. Large scale gustiness (global movement)
    let gust_noise = sin(time * speed * 0.2 + world_pos.x * 0.05 + world_pos.z * 0.03);
    let gust_strength = 1.0 + foliage_data.wind_gustiness * gust_noise;

    // 2. Trunk/Main branch sway (low frequency)
    let main_phase = dot(world_pos.xz, vec2<f32>(0.2, 0.15));
    let main_sway = sin(time * speed + main_phase) * amp * gust_strength;

    // 3. Cluster/Branch movement (medium frequency)
    // We use a combination of world and local pos to create "branch" groups
    let branch_seed = floor(world_pos * 2.0); // Create virtual clusters
    let branch_phase = hash13(branch_seed) * 6.28;
    let branch_sway = sin(time * speed * 1.7 + branch_phase) * amp * 0.4 * gust_strength;

    // 4. Individual leaf flutter (high frequency)
    // Small scale noise based on local position
    let leaf_noise = sin(time * speed * 4.5 + dot(local_pos, vec3<f32>(10.0, 12.0, 11.0)));
    let leaf_flutter = leaf_noise * foliage_data.wind_flutter * amp * gust_strength;

    // Combine directions
    let wind_dir = normalize(vec3<f32>(1.0, 0.1, 0.3));
    let side_dir = normalize(vec3<f32>(-0.3, 0.0, 1.0)); // Perpendicular-ish for some variety

    let total_displacement = (main_sway + branch_sway) * wind_dir + (leaf_flutter * side_dir);

    return total_displacement;
}

@vertex
fn vertex(vertex: Vertex) -> VertexOutput {
    var out: VertexOutput;

#ifdef SKINNED
    var world_from_local = bevy_pbr::skinning::skin_model(
        vertex.joint_indices,
        vertex.joint_weights,
        vertex.instance_index
    );
#else
    let world_from_local = mesh_functions::get_world_from_local(vertex.instance_index);
#endif

    var world_position = mesh_functions::mesh_position_local_to_world(
        world_from_local,
        vec4<f32>(vertex.position, 1.0)
    );
    // Wind displacement based on height (works in all passes now)
    let local_y = vertex.position.y;
    let height_factor = pow(saturate(local_y), 1.5);
    let displacement = wind_displacement(world_position.xyz, vertex.position) * height_factor;

    // Add a slight dip when swaying (simulating branch bending)
    let dip = length(displacement.xz) * 0.4 * height_factor;

    world_position = vec4<f32>(
        world_position.x + displacement.x,
        world_position.y + displacement.y - dip,
        world_position.z + displacement.z,
        1.0
    );

    out.position = position_world_to_clip(world_position.xyz);

#ifdef DEPTH_CLAMP_ORTHO
    out.clip_position_unclamped = out.position;
    out.position.z = min(out.position.z, 1.0);
#endif

#ifdef VERTEX_UVS_A
    out.uv = vertex.uv;
#endif

#ifdef VERTEX_UVS_B
    out.uv_b = vertex.uv_b;
#endif

#ifdef NORMAL_PREPASS_OR_DEFERRED_PREPASS
#ifdef SKINNED
    out.world_normal = bevy_pbr::skinning::skin_normals(world_from_local, vertex.normal);
#else
    out.world_normal = mesh_functions::mesh_normal_local_to_world(
        vertex.normal,
        vertex.instance_index
    );
#endif

#ifdef VERTEX_TANGENTS
    out.world_tangent = mesh_functions::mesh_tangent_local_to_world(
        world_from_local,
        vertex.tangent,
        vertex.instance_index
    );
#endif
#endif // NORMAL_PREPASS_OR_DEFERRED_PREPASS

#ifdef VERTEX_COLORS
    out.color = vertex.color;
#endif

#ifdef VERTEX_OUTPUT_INSTANCE_INDEX
    out.instance_index = vertex.instance_index;
#endif

#ifdef VISIBILITY_RANGE_DITHER
    out.visibility_range_dither = mesh_functions::get_visibility_range_dither_level(
        vertex.instance_index,
        world_from_local[3]
    );
#endif

// Forward-only outputs
#ifndef PREPASS_PIPELINE
    out.world_position = world_position;

#ifdef VERTEX_NORMALS
#ifdef SKINNED
    out.world_normal = bevy_pbr::skinning::skin_normals(world_from_local, vertex.normal);
#else
    out.world_normal = mesh_functions::mesh_normal_local_to_world(
        vertex.normal,
        vertex.instance_index
    );
#endif
#endif

#ifdef VERTEX_TANGENTS
    out.world_tangent = mesh_functions::mesh_tangent_local_to_world(
        world_from_local,
        vertex.tangent,
        vertex.instance_index
    );
#endif
#endif // !PREPASS_PIPELINE

    return out;
}

fn hash11(p: f32) -> f32 {
    var p_val = fract(p * 0.1031);
    p_val *= (p_val + 33.33);
    p_val *= (p_val + p_val);
    return fract(p_val);
}

fn get_leaf_mask(uv: vec2<f32>, instance_index: u32) -> f32 {
    let center = vec2<f32>(0.5, 0.5);
    let coords = (uv - center) * 2.0;
    let dist = length(coords);
    let angle = atan2(coords.y, coords.x);

    let seed = f32(instance_index);
    let rand = hash11(seed);

    // 1. Blade-like base shape (sharper tip)
    var shape = 0.7 + 0.25 * pow(abs(cos(angle * 0.5)), 0.5);

    // 2. Sharp "Hostile" Serration (spiky peaks instead of smooth waves)
    let serration_freq = 12.0 + rand * 8.0;
    let serration_wave = abs(sin(angle * serration_freq)) * 0.15;
    let serration = serration_wave * foliage_data.serration;

    // 3. Chaotic Fractal Irregularity
    let irr1 = sin(angle * 3.0 + rand * 6.28) * 0.15;
    let irr2 = sin(angle * 15.0 - seed) * 0.05;
    let irregularity = (irr1 + irr2) * foliage_data.irregularity;

    let final_radius = shape + serration + irregularity;

    // 4. Shredded / Slash patterns
    let slash_val = abs(sin(uv.x * 40.0 + uv.y * 30.0 + seed * 1.5));
    let slash_mask = smoothstep(0.95, 0.99, slash_val) * step(0.6, hash11(seed * 2.2));

    // 5. Random holes - reworked for visibility
    // Multi-frequency noise for varied hole sizes
    //let hole_n1 = sin(uv.x * 18.0 + seed * 0.7) * sin(uv.y * 22.0 - seed * 0.9);
    //let hole_n2 = sin(uv.x * 35.0 - seed) * sin(uv.y * 40.0 + seed * 1.3);
    //let hole_noise = (hole_n1 + hole_n2 * 0.5) * 0.5 + 0.5; // Normalize to [0, 1]
    //// holes=0 -> threshold=1.0 (no holes), holes=1 -> threshold=0.5 (many holes)
    //let hole_threshold = 1.0 - foliage_data.holes * 0.5;
    //let hole_mask = step(hole_threshold, hole_noise);

    // Combine into final mask
    var mask = 1.0 - smoothstep(final_radius - 0.02, final_radius + 0.02, dist);
    mask = saturate(mask - slash_mask * foliage_data.irregularity);
    //mask *= (1.0 - hole_mask);

    return mask;
}

#ifdef PREPASS_PIPELINE
@fragment
fn fragment(in: VertexOutput, @builtin(front_facing) is_front: bool) {
#ifdef VERTEX_UVS_A
#ifdef VERTEX_OUTPUT_INSTANCE_INDEX
    let mask = get_leaf_mask(in.uv, in.instance_index);
#else
    let mask = get_leaf_mask(in.uv, 0u);
#endif
    if (mask < 0.5) {
        discard;
    }
#endif

    prepass_alpha_discard(in);
}
#else
@fragment
fn fragment(
    in: VertexOutput,
    @builtin(front_facing) is_front: bool,
) -> FragmentOutput {
    var pbr_input = pbr_input_from_standard_material(in, is_front);

#ifdef VERTEX_OUTPUT_INSTANCE_INDEX
    let mask = get_leaf_mask(in.uv, in.instance_index);
#else
    let mask = get_leaf_mask(in.uv, 0u);
#endif

    pbr_input.material.base_color.a *= mask;

    // Edge drying effect: hostile dark-purple/bruised tone
    let edge_factor = smoothstep(0.0, 0.25, mask);
    let hostile_edge = vec4<f32>(0.15, 0.05, 0.12, pbr_input.material.base_color.a);
    let target_color = mix(hostile_edge, pbr_input.material.base_color, edge_factor);
    pbr_input.material.base_color = mix(pbr_input.material.base_color, target_color, foliage_data.edge_drying);

    pbr_input.material.base_color = alpha_discard(
        pbr_input.material,
        pbr_input.material.base_color
    );

    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);

    return out;
}
#endif
