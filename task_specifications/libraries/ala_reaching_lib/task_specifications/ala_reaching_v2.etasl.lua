require("context")
require("geometric")
require("math")
reqs = require("task_requirements")

task_description = [[
ALA reaching v2.

Adds:
- blended target input from /ala/blended_target
- feature variable s in [0, 1]
- straight adaptive path from initial TCP pose to blended target
- soft attraction of TCP to path

This is the first RAM/SIM composition step.
]]

linear_scale = constant(0.20)
task_frame_name = "tcp_frame"

robot = reqs.robot_model({
    task_frame_name,
})

task_frame = robot.getFrame(task_frame_name)

joystick_input = ctx:createInputChannelTwist("joystick_input")
blended_target = ctx:createInputChannelVector("ala_blended_target")

tcp_pos = origin(task_frame)
start_pos = initial_value(time, tcp_pos)

s = Variable{
    context = ctx,
    name = "s",
    vartype = "feature",
    initial = 0.0
}

-- Bound progress variable.
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

-- Straight path from start to blended target.
path_pos = start_pos + s * (blended_target - start_pos)

desired_vel_x = coord_x(transvel(joystick_input)) * linear_scale
desired_vel_y = coord_y(transvel(joystick_input)) * linear_scale
desired_vel_z = coord_z(transvel(joystick_input)) * linear_scale

-- Velocity SIM from user/HMI.
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

-- Soft path attraction. This is a first approximation of the RVGF/path RAM.
Constraint{
    context = ctx,
    name    = "ala_soft_path_attraction",
    expr    = tcp_pos - path_pos,
    K       = 2,
    weight  = 0.35,
    priority= 2
}

-- Keep orientation fixed.
Constraint{
    context = ctx,
    name    = "ala_keep_orientation",
    expr    = rotation(task_frame) * initial_value(time, rotation(task_frame)),
    K       = 4,
    weight  = 1,
    priority= 2
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
