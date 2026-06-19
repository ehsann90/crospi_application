require("context")
require("geometric")
require("math")
reqs = require("task_requirements")

task_description = [[
ALA reaching v4: RVGF plus automatic approach motion.

Adds:
- /ala/auto_cmd geometry_msgs/Point input
- auto_cmd.x > 0.5 activates automatic approach
- automatic progress of s toward 1
- automatic TCP attraction to the path f_p(s)

This approximates the paper's automatic reactive approach motion.
]]

linear_scale = constant(0.20)
task_frame_name = "tcp_frame"

tube_radius_start = constant(0.08)
tube_radius_end   = constant(0.02)
tube_shrink_start = constant(0.75)

rvgf_weight = constant(4.0)
s_damping_weight = constant(0.01)
cartesian_damping_weight = constant(0.02)

auto_s_speed = constant(0.12)
auto_s_weight = constant(4.0)
auto_follow_weight = constant(3.0)

robot = reqs.robot_model({
    task_frame_name,
})

task_frame = robot.getFrame(task_frame_name)

joystick_input = ctx:createInputChannelTwist("joystick_input")
blended_target = ctx:createInputChannelVector("ala_blended_target")
auto_cmd = ctx:createInputChannelVector("ala_auto_cmd")

auto_raw = coord_x(auto_cmd)
auto_enable = conditional(auto_raw - constant(0.5), constant(1.0), constant(0.0))

tcp_pos = origin(task_frame)
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

Constraint{
    context = ctx,
    name    = "s_damping",
    expr    = s,
    target  = constant(0.0) * time,
    K       = 0,
    weight  = s_damping_weight,
    priority= 2
}

path_pos = start_pos + s * (blended_target - start_pos)
path_vec = blended_target - start_pos

shrink_alpha_raw = (s - tube_shrink_start) / (constant(1.0) - tube_shrink_start)
shrink_alpha = conditional(shrink_alpha_raw, shrink_alpha_raw, constant(0.0))
rtube = tube_radius_start + shrink_alpha * (tube_radius_end - tube_radius_start)

desired_vel_x = coord_x(transvel(joystick_input)) * linear_scale
desired_vel_y = coord_y(transvel(joystick_input)) * linear_scale
desired_vel_z = coord_z(transvel(joystick_input)) * linear_scale

-- Manual velocity SIM.
Constraint{
    context = ctx,
    name    = "ala_x_velocity_sim",
    expr    = coord_x(tcp_pos),
    target  = desired_vel_x * time,
    K       = 0,
    weight  = 1,
    priority= 2
}

Constraint{
    context = ctx,
    name    = "ala_y_velocity_sim",
    expr    = coord_y(tcp_pos),
    target  = desired_vel_y * time,
    K       = 0,
    weight  = 1,
    priority= 2
}

Constraint{
    context = ctx,
    name    = "ala_z_velocity_sim",
    expr    = coord_z(tcp_pos),
    target  = desired_vel_z * time,
    K       = 0,
    weight  = 1,
    priority= 2
}

-- Drift reduction.
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

-- RVGF tube.
pos_err = tcp_pos - path_pos
distance_to_path = norm(pos_err)

-- Keep path_pos approximately at the closest point on the path to tcp_pos.
-- This reduces RVGF backlash/lag by forcing the error to be perpendicular
-- to the path tangent.
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
    weight  = 2.0,
    priority= 2
}

escape_margin = constant(0.06)
escape_end = rtube + escape_margin

-- Descending G3-like RVGF factor:
-- distance <= rtube       -> factor = 1
-- distance >= escape_end  -> factor = 0
-- between them            -> linearly decreases
rvgf_factor =
    conditional(
        distance_to_path - escape_end,
        constant(0.0),
        conditional(
            distance_to_path - rtube,
            (escape_end - distance_to_path) / escape_margin,
            constant(1.0)
        )
    )

rvgf_effective_weight = rvgf_weight * rvgf_factor

Constraint{
    context = ctx,
    name    = "rvgf_tube_constraint",
    expr    = distance_to_path,
    target_upper = rtube,
    K       = 3,
    weight  = rvgf_effective_weight,
    priority= 2
}

-- Automatic motion: drive s forward when auto mode is enabled.
Constraint{
    context = ctx,
    name    = "auto_progress_s",
    expr    = s,
    target  = auto_s_speed * time,
    K       = 0,
    weight  = auto_enable * auto_s_weight,
    priority= 2
}

-- Automatic motion: pull TCP toward the current point on f_p(s).
Constraint{
    context = ctx,
    name    = "auto_follow_path",
    expr    = tcp_pos - path_pos,
    K       = 2,
    weight  = auto_enable * auto_follow_weight,
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

finish_error = (constant(1.0) - s) + (constant(1.0) - auto_enable) * constant(10.0)

Monitor{
    context = ctx,
    name = "finish_after_auto_reaches_goal",
    lower = 0.005,
    actionname = "exit",
    expr = finish_error
}

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
ctx:setOutputExpression("target_x", coord_x(blended_target))
ctx:setOutputExpression("target_y", coord_y(blended_target))
ctx:setOutputExpression("target_z", coord_z(blended_target))
ctx:setOutputExpression("distance_to_path", distance_to_path)
ctx:setOutputExpression("rtube", rtube)
ctx:setOutputExpression("auto_enable", auto_enable)
ctx:setOutputExpression("rvgf_factor", rvgf_factor)
