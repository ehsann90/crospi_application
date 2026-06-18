require("context")
require("geometric")
require("math")
reqs = require("task_requirements")

task_description = [[
Minimal ALA reaching task, smoke-test version.

This version intentionally avoids reqs.parameters(), because direct
/crospi_node/readTaskSpecificationFile calls do not preload task parameters.
]]

-- Hard-coded smoke-test parameters.
linear_scale = constant(0.20)
execution_time = 0.0
task_frame_name = "tcp_frame"

robot = reqs.robot_model({
    task_frame_name,
})

task_frame = robot.getFrame(task_frame_name)

-- Must match the inputhandler varname in application_ala_reaching.setup.json.
joystick_input = ctx:createInputChannelTwist("joystick_input")

tcp_pos = origin(task_frame)

desired_vel_x = coord_x(transvel(joystick_input)) * linear_scale
desired_vel_y = coord_y(transvel(joystick_input)) * linear_scale
desired_vel_z = coord_z(transvel(joystick_input)) * linear_scale

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
