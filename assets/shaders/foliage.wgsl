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
}

@group(#{MATERIAL_BIND_GROUP}) @binding(100)
var<uniform> foliage_data: FoliageData;

fn wind_displacement(world_pos: vec3<f32>, time: f32) -> vec3<f32> {
    let wind_dir = vec3<f32>(1.0, 0.0, 0.3);

    let phase = dot(world_pos.xz, vec2<f32>(0.5, 0.3));

    let primary_wave = sin(time * foliage_data.wind_speed + phase);
    let secondary_wave = sin(time * foliage_data.wind_speed * 2.3 + phase * 1.7) * 0.3;
    let tertiary_wave = sin(time * foliage_data.wind_speed * 0.7 + phase * 1.7) * 0.5;

    let combined = (primary_wave + secondary_wave + tertiary_wave) * foliage_data.wind_amplitude;

    return normalize(wind_dir) * combined;
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

    // Wind displacement based on height
    let local_y = vertex.position.y;
    let height_factor = saturate(local_y * 2.0);
    let displacement = wind_displacement(world_position.xyz, globals.time) * height_factor;

    world_position = vec4<f32>(world_position.xyz + displacement, 1.0);

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

#ifdef PREPASS_PIPELINE
@fragment
fn fragment(in: VertexOutput, @builtin(front_facing) is_front: bool) -> FragmentOutput {
    prepass_alpha_discard(in);

    var out: FragmentOutput;

#ifdef DEPTH_CLAMP_ORTHO
    out.frag_depth = in.clip_position_unclamped.z;
#endif

#ifdef NORMAL_PREPASS
    out.normal = vec4(in.world_normal * 0.5 + vec3(0.5), 1.0);
#endif

#ifdef MOTION_VECTOR_PREPASS
    out.motion_vector = vec2(0.0);
#endif

    return out;
}
#else
@fragment
fn fragment(
    in: VertexOutput,
    @builtin(front_facing) is_front: bool,
) -> FragmentOutput {
    var pbr_input = pbr_input_from_standard_material(in, is_front);

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
