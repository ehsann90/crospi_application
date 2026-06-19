require("context")
require("geometric")
require("math")
reqs = require("task_requirements")

task_description = [[
ALA reaching v3: RVGF-like tube behavior.

This task approximates the Reactive Virtual Guidance Fixture from
Iregui et al. 2021.

Compared to v2:
- v2 always attracted TCP to the path centerline.
- v3 creates a tube around the path.
- Inside the tube, the user is mostly free.
- Outside the tube, the TCP is guided back toward the tube.

Still simplified:
- straight path only
- no PPCA/LfD path deformation yet
- no obstacle avoidance yet
- no automatic approach mode yet
]]

linear_scale = constant(0.20)
task_frame_name = "tcp_frame"

-- Tube parameters.
tube_radius_start = constant(0.08)  -- 8 cm tube near start/middle
tube_radius_end   = constant(0.02)  -- 2 cm tube near target
tube_shrink_start = constant(0.75)  -- shrink tube after 75 percent progress

-- Weights.
rvgf_weight = constant(4.0)
s_damping_weight = constant(0.01)
cartesian_damping_weight = constant(0.02)

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

-- Bound progress variable s.
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

-- Dampen s_dot. This corresponds roughly to the paper's constraint 12.
Constraint{
    context = ctx,
    name    = "s_damping",
    expr    = s,
    target  = constant(0.0) * time,
    K       = 0,
    weight  = s_damping_weight,
    priority= 2
}

-- Straight adaptive path from initial TCP to current blended target.
path_pos = start_pos + s * (blended_target - start_pos)
path_vec = blended_target - start_pos

-- Shrinking tube radius.
-- For s <= tube_shrink_start: radius = tube_radius_start
-- For s >  tube_shrink_start: radius linearly shrinks toward tube_radius_end.
shrink_alpha_raw = (s - tube_shrink_start) / (constant(1.0) - tube_shrink_start)
shrink_alpha = conditional(shrink_alpha_raw, shrink_alpha_raw, constant(0.0))
rtube = tube_radius_start + shrink_alpha * (tube_radius_end - tube_radius_start)

-- User velocity SIM.
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

-- Small damping to reduce drift when zero twist is sent.
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

-- RVGF-like tube constraint.
-- If TCP is inside the tube, the constraint is inactive.
-- If TCP exits the tube, it is guided back.
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

Constraint{
    context = ctx,
    name    = "rvgf_tube_constraint",
    expr    = distance_to_path,
    target_upper = rtube,
    K       = 3,
    weight  = rvgf_weight,
    priority= 2
}

-- Keep orientation fixed for now.
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
ctx:setOutputExpression("distance_to_path", distance_to_path)
ctx:setOutputExpression("rtube", rtube)
