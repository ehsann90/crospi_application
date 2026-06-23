require("context")
require("geometric")
require("math")
reqs = require("task_requirements")

task_description = [[
ALA reaching v5: paper-aligned G1/G2/G3 weight usage.

Weights are computed outside eTaSL by autonomy_manager_node.py.

G1:
  target arbitration is handled before this task by target_blender_node.

G2:
  automatic motion vs RVGF/manual guidance.

G3:
  distance-to-path weakens RVGF and s damping.

This task consumes:
  /spacenav/twist       -> joystick_input
  /ala/active_target    -> ala_blended_target
  /ala/motion_weights   -> ala_motion_weights
  /ala/auto_weights     -> ala_auto_weights
]]

linear_scale = constant(0.10)
task_frame_name = "tcp_frame"

tube_radius_start = constant(0.08)
tube_radius_end   = constant(0.02)
tube_shrink_start = constant(0.75)

auto_s_speed = constant(0.12)
cartesian_damping_weight = constant(0.01)

robot = reqs.robot_model({
    task_frame_name,
})

task_frame = robot.getFrame(task_frame_name)

joystick_input = ctx:createInputChannelTwist("joystick_input")
active_target = ctx:createInputChannelVector("ala_blended_target")

motion_weights = ctx:createInputChannelVector("ala_motion_weights")
auto_weights = ctx:createInputChannelVector("ala_auto_weights")

w_user_velocity = coord_x(motion_weights)
w_rvgf = coord_y(motion_weights)
w_s_damping = coord_z(motion_weights)

w_auto_progress = coord_x(auto_weights)
w_auto_follow = coord_y(auto_weights)
G2_from_weights = coord_z(auto_weights)

tcp_pos = origin(task_frame)

-- Always-active workspace safety constraints.
-- These are not modulated by G1/G2/G3.
-- Tune these bounds if your setup uses a different reachable workspace.
Constraint{
    context = ctx,
    name = "safe_x_min",
    expr = coord_x(tcp_pos),
    target_lower = constant(0.25),
    weight = constant(100.0),
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_x_max",
    expr = coord_x(tcp_pos),
    target_upper = constant(0.80),
    weight = constant(100.0),
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_y_min",
    expr = coord_y(tcp_pos),
    target_lower = constant(-0.45),
    weight = constant(100.0),
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_y_max",
    expr = coord_y(tcp_pos),
    target_upper = constant(0.45),
    weight = constant(100.0),
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_z_min",
    expr = coord_z(tcp_pos),
    target_lower = constant(0.10),
    weight = constant(100.0),
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_z_max",
    expr = coord_z(tcp_pos),
    target_upper = constant(0.65),
    weight = constant(100.0),
    priority = 1,
    K = constant(4.0)
}

start_pos = initial_value(time, tcp_pos)

s = Variable{
    context = ctx,
    name = "s",
    vartype = "feature",
    initial = 0.0
}

Constraint{
    context = ctx,
    name = "s_min",
    expr = s,
    target_lower = constant(0.0),
    weight = constant(20.0),
    priority = 2,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "s_max",
    expr = s,
    target_upper = constant(1.0),
    weight = constant(20.0),
    priority = 2,
    K = constant(4.0)
}

path_pos = start_pos + s * (active_target - start_pos)
path_vec = active_target - start_pos

pos_err = tcp_pos - path_pos
distance_to_path = norm(pos_err)

-- Keep path_pos approximately at the closest point on the path.
path_projection_error =
    coord_x(pos_err) * coord_x(path_vec) +
    coord_y(pos_err) * coord_y(path_vec) +
    coord_z(pos_err) * coord_z(path_vec)

Constraint{
    context = ctx,
    name    = "path_projection_constraint",
    expr    = path_projection_error,
    target  = constant(0.0),
    K       = 2,
    weight  = constant(2.0),
    priority= 2
}

-- G3 affects this through w_s_damping.
Constraint{
    context = ctx,
    name    = "s_damping",
    expr    = s,
    target  = constant(0.0) * time,
    K       = 0,
    weight  = w_s_damping,
    priority= 2
}

shrink_alpha_raw = (s - tube_shrink_start) / (constant(1.0) - tube_shrink_start)
shrink_alpha = conditional(shrink_alpha_raw, shrink_alpha_raw, constant(0.0))
rtube = tube_radius_start + shrink_alpha * (tube_radius_end - tube_radius_start)

desired_vel_x = coord_x(transvel(joystick_input)) * linear_scale
desired_vel_y = coord_y(transvel(joystick_input)) * linear_scale
desired_vel_z = coord_z(transvel(joystick_input)) * linear_scale

-- Table I constraint 7: user end-effector velocity.
-- Its weight is controlled by G2.
Constraint{
    context = ctx,
    name    = "ala_x_velocity_sim",
    expr    = coord_x(tcp_pos),
    target  = desired_vel_x * time,
    K       = 0,
    weight  = w_user_velocity,
    priority= 2
}

Constraint{
    context = ctx,
    name    = "ala_y_velocity_sim",
    expr    = coord_y(tcp_pos),
    target  = desired_vel_y * time,
    K       = 0,
    weight  = w_user_velocity,
    priority= 2
}

Constraint{
    context = ctx,
    name    = "ala_z_velocity_sim",
    expr    = coord_z(tcp_pos),
    target  = desired_vel_z * time,
    K       = 0,
    weight  = w_user_velocity,
    priority= 2
}

-- Small damping only to reduce numerical drift.
Constraint{
    context = ctx,
    name    = "tcp_x_damping",
    expr    = coord_x(tcp_pos),
    target  = constant(0.0) * time,
    K       = 0,
    weight  = cartesian_damping_weight,
    priority= 2
}

Constraint{
    context = ctx,
    name    = "tcp_y_damping",
    expr    = coord_y(tcp_pos),
    target  = constant(0.0) * time,
    K       = 0,
    weight  = cartesian_damping_weight,
    priority= 2
}

Constraint{
    context = ctx,
    name    = "tcp_z_damping",
    expr    = coord_z(tcp_pos),
    target  = constant(0.0) * time,
    K       = 0,
    weight  = cartesian_damping_weight,
    priority= 2
}

-- Table I constraint 11: RVGF tube.
-- Its weight is controlled by G2 and G3.
Constraint{
    context = ctx,
    name    = "rvgf_tube_constraint",
    expr    = distance_to_path,
    target_upper = rtube,
    K       = 2,
    weight  = w_rvgf,
    priority= 2
}

-- Table I constraint 13: automatic progress along path.
Constraint{
    context = ctx,
    name    = "auto_progress_s",
    expr    = s,
    target  = auto_s_speed * time,
    K       = 0,
    weight  = w_auto_progress,
    priority= 2
}

-- Table I constraint 14: automatic following of path.
Constraint{
    context = ctx,
    name    = "auto_follow_path",
    expr    = tcp_pos - path_pos,
    K       = 2,
    weight  = w_auto_follow,
    priority= 2
}

Constraint{
    context = ctx,
    name    = "ala_keep_orientation",
    expr    = rotation(task_frame) * initial_value(time, rotation(task_frame)),
    K       = 4,
    weight  = 1,
    priority= 2
}

-- Finish monitor removed in v6_safe_interactive so Crospi stays active during testing.

quat_tf = toQuat(rotation(task_frame))

ctx:setOutputExpression("time", time)
ctx:setOutputExpression("x_tf", coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf", coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf", coord_z(origin(task_frame)))
ctx:setOutputExpression("qx_tf", coord_x(vec(quat_tf)))
ctx:setOutputExpression("qy_tf", coord_y(vec(quat_tf)))
ctx:setOutputExpression("qz_tf", coord_z(vec(quat_tf)))
ctx:setOutputExpression("qw_tf", w(quat_tf))

ctx:setOutputExpression("desired_vel_x", desired_vel_x)
ctx:setOutputExpression("desired_vel_y", desired_vel_y)
ctx:setOutputExpression("desired_vel_z", desired_vel_z)

ctx:setOutputExpression("s", s)
ctx:setOutputExpression("path_x", coord_x(path_pos))
ctx:setOutputExpression("path_y", coord_y(path_pos))
ctx:setOutputExpression("path_z", coord_z(path_pos))
ctx:setOutputExpression("target_x", coord_x(active_target))
ctx:setOutputExpression("target_y", coord_y(active_target))
ctx:setOutputExpression("target_z", coord_z(active_target))
ctx:setOutputExpression("distance_to_path", distance_to_path)
ctx:setOutputExpression("rtube", rtube)
ctx:setOutputExpression("G2_from_weights", G2_from_weights)
ctx:setOutputExpression("w_user_velocity", w_user_velocity)
ctx:setOutputExpression("w_rvgf", w_rvgf)
ctx:setOutputExpression("w_s_damping", w_s_damping)
ctx:setOutputExpression("w_auto_progress", w_auto_progress)
ctx:setOutputExpression("w_auto_follow", w_auto_follow)
