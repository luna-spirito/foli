use std::f32::consts::PI;

use bevy::prelude::*;

const STEP_THRESHOLD: f32 = 0.01;
const ROTATION_SPEED: f32 = 10.0;

#[derive(Component)]
pub struct DebugShowAxis;

pub fn debug_show_axis(q: Query<&GlobalTransform, With<DebugShowAxis>>, mut gizmos: Gizmos) {
    for gtr in q {
        let pos = gtr.translation();
        gizmos.arrow(pos, pos + gtr.back().as_vec3(), Color::srgb(0.0, 0.0, 1.0)); // Z
        gizmos.arrow(pos, pos + gtr.up().as_vec3(), Color::srgb(0.0, 1.0, 0.0)); // Y
        gizmos.arrow(pos, pos + gtr.right().as_vec3(), Color::srgb(1.0, 0.0, 0.0)); // Y
    }
}

#[derive(Component)]
pub struct BipedalCfg {
    pub speed: f32,
    pub step_duration: f32,
    pub step_height: f32,
    pub ankle_height: f32,
    pub torso_off_min: f32,
    pub torso_off_sway: f32,
}

#[derive(Component)]
pub struct Bipedal {
    legs: [Leg; 2], // 0: Left, 1: Right
    step: Option<StepState>,
}

#[derive(Clone)]
struct Leg {
    hip: Entity,
    hip_rot: Quat,
    knee: Entity,
    knee_rot: Quat,
    ankle: Entity,
    ankle_rot: Quat,
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
    progress: f32,
}

pub fn setup_bipedal(
    mut commands: Commands,
    walker_query: Query<(Entity, &Transform), (With<BipedalCfg>, Without<Bipedal>)>,
    children_query: Query<&Children>,
    name_query: Query<&Name>,
    bone_transforms: Query<(&GlobalTransform, &Transform), Without<BipedalCfg>>,
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
            let get_pos = |e| {
                bone_transforms
                    .get(e)
                    .map(|t| (t.0.translation(), t.1.rotation))
            };

            if let (Ok(lh), Ok(lk), Ok(la), Ok(rh), Ok(rk), Ok(ra)) = (
                get_pos(l_hip),
                get_pos(l_knee),
                get_pos(l_ankle),
                get_pos(r_hip),
                get_pos(r_knee),
                get_pos(r_ankle),
            ) {
                let l_len = (lh.0.distance(lk.0), lk.0.distance(la.0));
                let r_len = (rh.0.distance(rk.0), rk.0.distance(ra.0));

                // Initialize feet directly under where they are (or idealized)
                // For simplicity, let's start them at the ground relative to current transform
                let forward = transform.forward();
                let right = transform.right();

                let l_start = transform.translation + (-right * 0.15) + (forward * 0.2);
                let r_start = transform.translation + (right * 0.15) + (forward * -0.2); // Staggered start

                let legs = [
                    Leg {
                        hip: l_hip,
                        hip_rot: lh.1,
                        knee: l_knee,
                        knee_rot: lk.1,
                        ankle: l_ankle,
                        ankle_rot: la.1,
                        lengths: l_len,
                        current_pos: Vec3::new(l_start.x, 0.0, l_start.z),
                        side_offset: -0.15,
                    },
                    Leg {
                        hip: r_hip,
                        hip_rot: rh.1,
                        knee: r_knee,
                        knee_rot: rk.1,
                        ankle: r_ankle,
                        ankle_rot: ra.1,
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
    mut query: Query<(
        Entity,
        &mut Transform,
        &BipedalCfg,
        &mut Bipedal,
        &GlobalTransform,
    )>,
    mut transforms: Query<&mut Transform, Without<Bipedal>>,
    global_transforms: Query<&GlobalTransform>,
    parents: Query<&ChildOf>,
    mut gizmos: Gizmos,
) {
    for (entity, mut transform, bipedal_cfg, mut bipedal, global_transform) in &mut query {
        let dt = time.delta_secs();

        // 1. Move Body
        let move_dir = if let Some(target) = target_opt {
            let to_target = target - transform.translation;
            let dist = to_target.length();

            if dist > 0.01 {
                // Direction to target (ignoring Y for ground movement)
                let dir_flat = Vec3::new(to_target.x, 0.0, to_target.z).normalize_or_zero();

                if dir_flat != Vec3::ZERO {
                    // Rotate towards target
                    let target_rot = Transform::default().looking_at(dir_flat, Vec3::Y).rotation;
                    let angle = transform.rotation.angle_between(target_rot);

                    // Ensure constant rotation speed (angular velocity)
                    // standard slerp(rot, target, t) with fixed t is non-linear (slows down at end).
                    // By calculating t = (speed * dt) / angle, we move a fixed arc length each frame.
                    if angle > 0.0 {
                        let max_angle = ROTATION_SPEED * dt;
                        let t = (max_angle / angle).min(1.0);
                        transform.rotation = transform.rotation.slerp(target_rot, t);
                    }

                    // Move forward (along new facing)
                    let velocity = dir_flat * bipedal_cfg.speed * dt;
                    transform.translation += velocity;
                }
                dir_flat
            } else {
                Vec3::ZERO
            }
        } else {
            Vec3::ZERO
        };
        transform.translation.y = -0.2;

        let body_pos = transform.translation;
        let body_right = transform.right();
        let body_right_vec = Vec3::new(body_right.x, 0.0, body_right.z).normalize();

        // 2. Handle Stepping Logic
        if let Bipedal {
            step: Some(step),
            legs,
        } = &mut *bipedal
        {
            step.progress += dt / bipedal_cfg.step_duration;
            let t = step.progress.clamp(0.0, 1.0);

            // Dynamic Step Retargeting:
            // If the input changes (move_dir changes), we want to land the foot
            // at a new, more appropriate location (e.g. stop earlier).
            // We avoid updating near the end (t > 0.9) to prevent numerical instability.
            if t < 0.9 {
                let remaining_time = bipedal_cfg.step_duration * (1.0 - t);
                // Predict where body will be at the end of the step with *current* velocity
                let predicted_body_end_pos =
                    body_pos + (move_dir * bipedal_cfg.speed * remaining_time);

                let idx = step.leg_index;
                let leg_side_offset = legs[idx].side_offset;

                let new_target =
                    predicted_body_end_pos + (body_right_vec * leg_side_offset) + (move_dir * 0.25);
                let new_target_ground = Vec3::new(new_target.x, 0.0, new_target.z);

                // We need to switch the bezier curve from (OldStart -> OldTarget) to (NewStart -> NewTarget)
                // such that at current 't', the position is identical (no pop).
                // Pos(t) = Start * (1 - smooth_t) + Target * smooth_t
                // CurrentPos = NewStart * (1 - smooth_t) + NewTarget * smooth_t
                // NewStart = (CurrentPos - NewTarget * smooth_t) / (1 - smooth_t)

                let smooth_t = t * t * (3.0 - 2.0 * t);
                let current_ground = step.start_pos.lerp(step.target_pos, smooth_t);

                let denom = 1.0 - smooth_t;
                if denom > 1e-4 {
                    step.start_pos = (current_ground - new_target_ground * smooth_t) / denom;
                    step.target_pos = new_target_ground;
                }
            }

            // Smoothstep for smoother motion
            let smooth_t = t * t * (3.0 - 2.0 * t);

            let ground_curr = step.start_pos.lerp(step.target_pos, smooth_t);
            let height = (t * PI).sin() * bipedal_cfg.step_height;

            legs[step.leg_index].current_pos = ground_curr + Vec3::Y * height;

            if t >= 1.0 {
                // Finish step
                legs[step.leg_index].current_pos.y = 0.0;
                bipedal.step = None;
            }
        } else {
            let mut best_candidate = None;
            let mut max_dist = 0.0;

            for (i, leg) in bipedal.legs.iter().enumerate() {
                // Calculate ideal position for this leg
                let ideal_pos = body_pos + (body_right_vec * leg.side_offset) + (move_dir * 0.3);
                let ideal_ground = Vec3::new(ideal_pos.x, 0.0, ideal_pos.z);

                let dist = leg.current_pos.distance(ideal_ground);

                if dist > STEP_THRESHOLD && dist > max_dist {
                    max_dist = dist;
                    best_candidate = Some(i);
                }
            }

            if let Some(idx) = best_candidate {
                // Start step
                // Predict where the body will be when step finishes
                let predicted_body_pos =
                    body_pos + (move_dir * bipedal_cfg.speed * bipedal_cfg.step_duration);

                // Target is relative to predicted body pos
                let target_pos = predicted_body_pos
                    + (body_right_vec * bipedal.legs[idx].side_offset)
                    + (move_dir * 0.25);

                bipedal.step = Some(StepState {
                    leg_index: idx,
                    start_pos: bipedal.legs[idx].current_pos,
                    target_pos: Vec3::new(target_pos.x, 0.0, target_pos.z),
                    progress: 0.0,
                });
            }
        }

        // Debug Gizmos
        for leg in &bipedal.legs {
            gizmos.sphere(leg.current_pos, 0.05, Color::srgb(1.0, 1.0, 0.0));
        }
    }
}

pub fn apply_ik(
    bipedal_query: Query<(&Bipedal, &BipedalCfg, &GlobalTransform)>,
    mut bone_query: Query<(&ChildOf, &mut Transform, &GlobalTransform)>,
) {
    for (bipedal, bipedal_cfg, root_gtr) in bipedal_query {
        for leg in &bipedal.legs {
            let Ok([mut hip, mut knee, mut ankle]) =
                bone_query.get_many_mut([leg.hip, leg.knee, leg.ankle])
            else {
                continue;
            };

            let target = root_gtr.rotation().inverse().mul_vec3(
                leg.current_pos + Vec3::Y * bipedal_cfg.ankle_height - hip.2.translation(),
            );
            let (hip_ik_rot, knee_ik_rot) = solve_ik(target, leg.lengths);

            hip.1.rotation = hip_ik_rot * leg.hip_rot;
            knee.1.rotation = knee_ik_rot * leg.knee_rot;

            // TODO: REDO
            // 3. The Ankle Math (The "Cancellation" logic)
            // This represents the orientation of the foot relative to the pelvis in bind pose
            let original_chain = leg.hip_rot * leg.knee_rot * leg.ankle_rot;
            // This cancels the new IK movement and applies the original orientation
            let ankle_new = (hip.1.rotation * knee.1.rotation).inverse() * original_chain;
            ankle.1.rotation = ankle_new;
        }
    }
}

pub fn solve_ik(target: Vec3, (l1, l2): (f32, f32)) -> (Quat, Quat) {
    let dist = target.length();
    let dist_sq = dist * dist;

    // 1. Law of Cosines for internal angles
    let hip_angle = ((l1 * l1 + dist_sq - l2 * l2) / (2.0 * l1 * dist))
        .clamp(-1.0, 1.0)
        .acos();
    let knee_angle = ((l1 * l1 + l2 * l2 - dist_sq) / (2.0 * l1 * l2))
        .clamp(-1.0, 1.0)
        .acos();

    (
        Quat::from_rotation_x(hip_angle) * Quat::from_rotation_arc(Vec3::NEG_Y, target.normalize()),
        Quat::from_rotation_x(PI - knee_angle),
    )
}

// pub fn solve_ik(target: Vec3, pole: Vec3, (l1, l2): (f32, f32)) -> (Quat, Quat) {
//     let dist = target.length();

//     let hip_angle = ((l1 * l1 + dist * dist - l2 * l2) / (2.0 * l1 * dist))
//         .clamp(-1.0, 1.0)
//         .acos();
//     let knee_angle = ((l1 * l1 + l2 * l2 - dist * dist) / (2.0 * l1 * l2))
//         .clamp(-1.0, 1.0)
//         .acos();
//     println!("{}", knee_angle * 180.0 / PI);

//     let norm = target.cross(pole);
//     (
//         Quat::from_axis_angle(norm, hip_angle + Vec3::NEG_Y.angle_between(target)),
//         Quat::from_rotation_x(PI - knee_angle),
//     )
// }

// pub fn solve_ik(target: Vec3, pole: Vec3, (l1, l2): (f32, f32)) -> (Quat, Quat) {
//     let dist = target.length();

//     // Safety check to prevent NaN if target is at (0,0,0)
//     if dist < 0.001 {
//         return (Quat::IDENTITY, Quat::IDENTITY);
//     }

//     // 1. Law of Cosines (Magnitude)
//     // Calculate the internal angles required to form the triangle
//     let cos_hip = (l1 * l1 + dist * dist - l2 * l2) / (2.0 * l1 * dist);
//     let hip_angle_tri = cos_hip.clamp(-1.0, 1.0).acos();

//     let cos_knee = (l1 * l1 + l2 * l2 - dist * dist) / (2.0 * l1 * l2);
//     let knee_angle_tri = cos_knee.clamp(-1.0, 1.0).acos();

//     // 2. Aim the Leg (Global Orientation)
//     // Create a rotation that points the leg's default direction (Down/-Y)
//     // directly at the target vector.
//     let target_dir = target.normalize();
//     let aim_rot = Quat::from_rotation_arc(Vec3::NEG_Y, target_dir);

//     // 3. Determine the Bend Plane
//     // Calculate the axis perpendicular to the plane defined by Target and Pole.
//     // If you want knees to bend forward, `pole` should be Vec3::Z (or whatever represents forward).
//     // Note: If target and pole are parallel, this is unstable, so we fallback to X.
//     let bend_axis = if target_dir.abs_diff_eq(pole.normalize(), 1e-4) {
//         Vec3::X
//     } else {
//         pole.cross(target_dir).normalize()
//     };

//     // 4. Construct Rotations
//     // Apply the hip angle to bend the thigh away from the direct target line
//     // to accommodate the knee.
//     // The order is important: We rotate around the bend_axis *relative* to the aim.
//     let hip_bend_rot = Quat::from_axis_angle(bend_axis, hip_angle_tri);

//     // Combine: First Aim, then Bend.
//     // (In quaternion math `A * B` applies B then A, but since our axis is derived
//     // from the target space, we treat this as a composition).
//     let hip_rot = hip_bend_rot * aim_rot;

//     // Knee Rotation:
//     // Purely internal bend. We assume standard rig where X is the bend axis.
//     // (PI - angle) because 180 degrees is a straight leg, 90 is bent.
//     let knee_rot = Quat::from_rotation_x(PI - knee_angle_tri);

//     (hip_rot, knee_rot)
// }
