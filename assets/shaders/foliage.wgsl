#import bevy_pbr::mesh_functions::{get_world_from_local, mesh_position_local_to_clip}
#import bevy_pbr::forward_io::VertexOutput
#import bevy_pbr::mesh_view_bindings::globals

struct FoliageData {
    wind_speed: f32,
    wind_amplitude: f32,
}

@group(#{MATERIAL_BIND_GROUP}) @binding(100) var<uniform> extension: FoliageData;

// We hook into the vertex shader to apply wind displacement
#import bevy_pbr::forward_io::Vertex

@vertex
fn vertex(vertex: Vertex) -> VertexOutput {
    var out: VertexOutput;

    var world_from_local = get_world_from_local(vertex.instance_index);
    var world_position = world_from_local * vec4<f32>(vertex.position, 1.0);

    // Simple wind sway logic
    let time = globals.time * extension.wind_speed;
    let sway = sin(time + world_position.x + world_position.z) * extension.wind_amplitude;

    world_position.x += sway;
    world_position.z += sway * 0.5;

    // Use modified world position for clip projection
    // Note: mesh_position_local_to_clip usually takes local pos.
    // If we want to use world pos, we'd need a different function.
    // However, it's easier to just Sway local pos.
    var local_pos = vertex.position;
    // We can sway local pos instead
    local_pos.x += sway;
    local_pos.z += sway * 0.5;

    out.position = mesh_position_local_to_clip(world_from_local, vec4<f32>(local_pos, 1.0));

    // Pass everything else to PBR
    out.world_position = world_position;
    out.world_normal = vertex.normal;
    out.uv = vertex.uv;

    return out;
}

#import bevy_pbr::pbr_fragment::pbr_input_from_standard_material
#import bevy_pbr::pbr_functions::{alpha_discard, apply_pbr_lighting, main_pass_post_lighting_processing}
#import bevy_pbr::forward_io::{FragmentOutput}

@fragment
fn fragment(
    in: VertexOutput,
    @builtin(front_facing) is_front: bool,
) -> FragmentOutput {
    // Generate PbrInput
    var pbr_input = pbr_input_from_standard_material(in, is_front);

    // Soft UV edges to hide card boundaries
    let uv_mask = smoothstep(0.0, 0.1, in.uv.x) * (1.0 - smoothstep(0.9, 1.0, in.uv.x)) *
                  smoothstep(0.0, 0.1, in.uv.y) * (1.0 - smoothstep(0.9, 1.0, in.uv.y));
    pbr_input.material.base_color.a *= uv_mask;

    // Alpha discount
    pbr_input.material.base_color = alpha_discard(pbr_input.material, pbr_input.material.base_color);

    var out: FragmentOutput;

    // Minimal standard PBR
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);

    return out;
}
