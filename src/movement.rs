use std::f32::consts::PI;

use bevy::prelude::*;

#[derive(Component)]
pub struct BipedalCfg {
    pub speed: f32,
}

#[derive(Component)]
pub struct Bipedal {
    legs: [Leg; 2], // 0: Left, 1: Right
    step: Option<StepState>,
}

#[derive(Clone)]
struct Leg {
    hip: Entity,
    knee: Entity,
    ankle: Entity,
    lengths: (f32, f32),
    // The current world-space position of the foot (or where it's trying to be)
    current_pos: Vec3,
    // Lateral offset from center (e.g. -0.15 for left, +0.15 for right)
    side_offset: f32,
}

struct StepState {
    leg_index: usize,
    start_pos: Vec3,
    target_pos: Vec3,
    start_time: f32,
    duration: f32,
}

pub fn setup_bipedal(
    mut commands: Commands,
    walker_query: Query<(Entity, &Transform), (With<BipedalCfg>, Without<Bipedal>)>,
    children_query: Query<&Children>,
    name_query: Query<&Name>,
    global_transforms: Query<&GlobalTransform>,
) {
    for (entity, transform) in &walker_query {
        let mut left_bones = (None, None, None);
        let mut right_bones = (None, None, None);

        let mut stack = vec![entity];
        while let Some(curr) = stack.pop() {
            if let Ok(name) = name_query.get(curr) {
                match name.as_str() {
                    "mixamorig:LeftUpLeg" => left_bones.0 = Some(curr),
                    "mixamorig:LeftLeg" => left_bones.1 = Some(curr),
                    "mixamorig:LeftFoot" => left_bones.2 = Some(curr),
                    "mixamorig:RightUpLeg" => right_bones.0 = Some(curr),
                    "mixamorig:RightLeg" => right_bones.1 = Some(curr),
                    "mixamorig:RightFoot" => right_bones.2 = Some(curr),
                    _ => {}
                }
            }
            if let Ok(children) = children_query.get(curr) {
                stack.extend(children.iter());
            }
        }

        if let (
            Some(l_hip),
            Some(l_knee),
            Some(l_ankle),
            Some(r_hip),
            Some(r_knee),
            Some(r_ankle),
        ) = (
            left_bones.0,
            left_bones.1,
            left_bones.2,
            right_bones.0,
            right_bones.1,
            right_bones.2,
        ) {
            // Calculate lengths
            let get_pos = |e| global_transforms.get(e).map(|t| t.translation());

            if let (Ok(lh), Ok(lk), Ok(la), Ok(rh), Ok(rk), Ok(ra)) = (
                get_pos(l_hip),
                get_pos(l_knee),
                get_pos(l_ankle),
                get_pos(r_hip),
                get_pos(r_knee),
                get_pos(r_ankle),
            ) {
                let l_len = (lh.distance(lk), lk.distance(la));
                let r_len = (rh.distance(rk), rk.distance(ra));

                // Initialize feet directly under where they are (or idealized)
                // For simplicity, let's start them at the ground relative to current transform
                let forward = transform.forward();
                let right = transform.right();

                let l_start = transform.translation + (-right * 0.15) + (forward * 0.2);
                let r_start = transform.translation + (right * 0.15) + (forward * -0.2); // Staggered start

                let legs = [
                    Leg {
                        hip: l_hip,
                        knee: l_knee,
                        ankle: l_ankle,
                        lengths: l_len,
                        current_pos: Vec3::new(l_start.x, 0.0, l_start.z),
                        side_offset: -0.15,
                    },
                    Leg {
                        hip: r_hip,
                        knee: r_knee,
                        ankle: r_ankle,
                        lengths: r_len,
                        current_pos: Vec3::new(r_start.x, 0.0, r_start.z),
                        side_offset: 0.15,
                    },
                ];

                commands.entity(entity).insert(Bipedal { legs, step: None });
                println!("Walker Rig Initialized");
            }
        }
    }
}

pub fn locomotion(
    In(target_opt): In<Option<Vec3>>,
    time: Res<Time>,
    mut query: Query<(&mut Transform, &BipedalCfg, &mut Bipedal)>,
    mut gizmos: Gizmos,
) {
    for (mut transform, bipedal_cfg, mut bipedal) in &mut query {
        let dt = time.delta_secs();
        let mut move_dir = None;

        // 1. Move Body
        if let Some(target) = target_opt {
            let to_target = target - transform.translation;
            let dist = to_target.length();

            if dist > 0.01 {
                // Direction to target (ignoring Y for ground movement)
                let dir_flat = Vec3::new(to_target.x, 0.0, to_target.z).normalize_or_zero();

                if dir_flat != Vec3::ZERO {
                    // Rotate towards target
                    let target_rot = Transform::default().looking_at(target, Vec3::Y).rotation;
                    transform.rotation = transform.rotation.slerp(target_rot, 10.0 * dt);

                    // Move forward (along new facing)
                    // Or move directly towards target?
                    // "Moves towards it" - let's move along the calculated direction
                    let velocity = dir_flat * bipedal_cfg.speed * dt;
                    transform.translation += velocity;

                    move_dir = Some(dir_flat);
                }
            }
        }

        let body_pos = transform.translation;
        let body_right = transform.right();
        let body_right_vec = Vec3::new(body_right.x, 0.0, body_right.z).normalize();

        // 2. Handle Stepping Logic

        // If stepping, update the animation
        if let Bipedal {
            step: Some(step),
            legs,
        } = &mut *bipedal
        {
            let elapsed = time.elapsed_secs() - step.start_time;
            let t = (elapsed / step.duration).clamp(0.0, 1.0);

            // Smoothstep for smoother motion
            let smooth_t = t * t * (3.0 - 2.0 * t);

            let ground_curr = step.start_pos.lerp(step.target_pos, smooth_t);
            let height = (t * PI).sin() * 0.2; // 20cm step height

            legs[step.leg_index].current_pos = ground_curr + Vec3::Y * height;

            if t >= 1.0 {
                // Finish step
                legs[step.leg_index].current_pos.y = 0.0;
                bipedal.step = None;
            }
        } else if let Some(move_dir) = move_dir {
            // No step active, check if we need to start one
            // Ideally, we want to step with the leg that is furthest behind or most uncomfortable

            let mut best_candidate = None;
            let mut max_dist = 0.0;

            // Step threshold: when foot is this far from "ideal" position, trigger step
            let step_threshold = 0.45;

            for (i, leg) in bipedal.legs.iter().enumerate() {
                // Calculate ideal position for this leg
                // Ideal is: BodyPos + SideOffset + slight forward lead
                let ideal_pos = body_pos + (body_right_vec * leg.side_offset) + (move_dir * 0.3);
                let ideal_ground = Vec3::new(ideal_pos.x, 0.0, ideal_pos.z);

                let dist = leg.current_pos.distance(ideal_ground);

                if dist > step_threshold && dist > max_dist {
                    max_dist = dist;
                    best_candidate = Some(i);
                }
            }

            if let Some(idx) = best_candidate {
                // Start step
                // Predict where the body will be when step finishes
                let step_duration = 0.35;
                let predicted_body_pos = body_pos + (move_dir * bipedal_cfg.speed * step_duration);

                // Target is relative to predicted body pos
                // Add some "overstep" to place foot ahead of body
                let target_pos = predicted_body_pos
                    + (body_right_vec * bipedal.legs[idx].side_offset)
                    + (move_dir * 0.25);

                bipedal.step = Some(StepState {
                    leg_index: idx,
                    start_pos: bipedal.legs[idx].current_pos,
                    target_pos: Vec3::new(target_pos.x, 0.0, target_pos.z),
                    start_time: time.elapsed_secs(),
                    duration: step_duration,
                });
            }
        }
        // If not moving and no step active, we just stand.
        // Optional: Add logic to bring feet back to neutral if stopped for a while.

        // Debug Gizmos
        for leg in &bipedal.legs {
            gizmos.sphere(leg.current_pos, 0.05, Color::srgb(1.0, 1.0, 0.0));
        }
    }
}

pub fn apply_ik(
    mut bipedal_query: Query<(&Bipedal, &Transform, &GlobalTransform)>,
    mut transforms: Query<&mut Transform, Without<Bipedal>>,
    global_transforms: Query<&GlobalTransform>,
    parents: Query<&ChildOf>,
) {
    for (bipedal, _root_local, root_global) in &mut bipedal_query {
        let pole = root_global.forward(); // Knees point forward

        for leg in &bipedal.legs {
            // Get current hip global pos
            let hip_global = match global_transforms.get(leg.hip) {
                Ok(t) => t.translation(),
                Err(_) => continue,
            };

            // Solve IK (Pure Math)
            let (hip_rot_global, knee_rot_local) = solve_two_bone_ik(
                hip_global,
                leg.current_pos,
                pole.into(),
                leg.lengths.0,
                leg.lengths.1,
            );

            // Apply Knee (Local)
            if let Ok(mut knee_tf) = transforms.get_mut(leg.knee) {
                knee_tf.rotation = knee_rot_local;
            }

            // Apply Hip (Global -> Local)
            // Need to convert global rotation to local space of the hip's parent
            if let Ok(parent) = parents.get(leg.hip) {
                if let Ok(parent_global) = global_transforms.get(parent.parent()) {
                    let (_, parent_rot, _) = parent_global.to_scale_rotation_translation();
                    let hip_local_rot = parent_rot.inverse() * hip_rot_global;

                    if let Ok(mut hip_tf) = transforms.get_mut(leg.hip) {
                        hip_tf.rotation = hip_local_rot;
                    }
                }
            }
        }
    }
}

/// Pure function: Solves 2-Bone IK
/// Returns: (Hip Global Rotation, Knee Local Rotation)
pub fn solve_two_bone_ik(
    hip_pos: Vec3,
    target_pos: Vec3,
    pole_vector: Vec3, // Direction knees should point (usually forward)
    l1: f32,           // Thigh length
    l2: f32,           // Shin length
) -> (Quat, Quat) {
    let to_target = target_pos - hip_pos;
    let dist = to_target.length();

    // 1. Knee Angle (Law of Cosines)
    // Clamp distance so we don't break acos
    let dist_clamped = dist.clamp(0.01, l1 + l2 - 0.001);

    let cos_knee = (l1 * l1 + l2 * l2 - dist_clamped * dist_clamped) / (2.0 * l1 * l2);
    let angle_knee = cos_knee.clamp(-1.0, 1.0).acos();

    // Knee bend: Assuming standard rigging where -X rotation bends knee backwards?
    // In many rigs (Mixamo), 0 rotation is straight leg.
    // We want to rotate along the local X axis.
    // The angle calculated is the internal angle.
    // If leg is straight, angle is PI (180). We want 0 rotation.
    // If leg is bent 90 deg, angle is PI/2. We want -90 deg rotation.
    // So rotation is -(PI - angle).
    let knee_rot_local = Quat::from_rotation_x(-(PI - angle_knee));

    // 2. Hip Orientation
    // We need to rotate the thigh so that:
    // a) The ankle ends up at target_pos
    // b) The knee points in direction of pole_vector

    // First, calculate the angle offset for the thigh triangle
    let cos_hip_offset =
        (l1 * l1 + dist_clamped * dist_clamped - l2 * l2) / (2.0 * l1 * dist_clamped);
    let angle_hip_offset = cos_hip_offset.clamp(-1.0, 1.0).acos();

    // Base direction to target
    let target_dir = to_target.normalize();

    // Plane Normal: The plane formed by the leg triangle.
    // Defined by the target vector and the pole vector.
    // "Knee should point roughly towards pole"
    let plane_normal = target_dir.cross(pole_vector).normalize_or_zero();

    // If pole and target are collinear, pick a default (e.g. Up)
    let plane_normal = if plane_normal.length_squared() < 0.001 {
        target_dir.cross(Vec3::Y).normalize_or_zero()
    } else {
        plane_normal
    };

    // Rotate the target vector by the hip offset angle around the plane normal
    // This gives us the direction the thigh bone should point in global space.
    let thigh_dir = Quat::from_axis_angle(plane_normal, angle_hip_offset).mul_vec3(target_dir);

    // Now construct a rotation looking down thigh_dir, with Up roughly towards plane_normal?
    // Actually, usually:
    // -Y axis (bone length) points to child.
    // But in Mixamo/Blender, usually +Y or -Z points down the bone.
    // Let's assume standard Bevy/GLTF: Bones often point along +Y or something.
    // Wait, previous code used specific basis construction.
    // Let's assume:
    // Thigh Forward (-Z or Y?) -> thigh_dir
    // Thigh Right (X) -> plane_normal

    // Let's try constructing a LookAt.
    // Assuming the bone's local Y axis points down the leg (common in Blender export).
    // And X is the axis of rotation for the knee.
    let y_axis = thigh_dir.normalize();
    let x_axis = plane_normal.normalize();
    let z_axis = x_axis.cross(y_axis).normalize();

    // Mat3 columns are (X, Y, Z)
    let hip_global_rot = Quat::from_mat3(&Mat3::from_cols(x_axis, y_axis, z_axis));

    (hip_global_rot, knee_rot_local)
}
