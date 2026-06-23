require("context")
require("geometric")
require("math")
reqs = require("task_requirements")

task_description = [[
Adaptive-level-autonomy reaching task with user velocity input, RVGF tube guidance,
automatic path progress, and automatic path following. The target and autonomy
weights are provided through ROS input channels.
]]

param = reqs.parameters(task_description,{
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in Cartesian space", default="tcp_frame", required=false}),

    reqs.params.scalar({name="linear_scale", description="Scale applied to translational HMI/twist velocity", default=0.10, required=false, minimum=0.0}),
    reqs.params.scalar({name="rtube_start", description="Initial RVGF tube radius [m]", default=0.08, required=false, minimum=0.0}),
    reqs.params.scalar({name="rtube_end", description="Final RVGF tube radius near target [m]", default=0.02, required=false, minimum=0.0}),
    reqs.params.scalar({name="tube_shrink_start", description="Path progress value at which tube starts shrinking", default=0.75, required=false, minimum=0.0, maximum=1.0}),
    reqs.params.scalar({name="auto_s_speed", description="Automatic progress speed for path coordinate s", default=0.20, required=false, minimum=0.0}),

    reqs.params.scalar({name="cartesian_damping_weight", description="Cartesian damping weight", default=0.0, required=false, minimum=0.0}),
    reqs.params.scalar({name="path_projection_base_weight", description="Constant part of path projection weight", default=0.0, required=false, minimum=0.0}),
    reqs.params.scalar({name="orientation_weight", description="Orientation hold weight", default=0.0, required=false, minimum=0.0}),
    reqs.params.scalar({name="workspace_weight", description="Workspace safety constraint weight", default=100.0, required=false, minimum=0.0}),

    reqs.params.scalar({name="workspace_x_min", description="Minimum workspace x [m]", default=-0.35, required=false}),
    reqs.params.scalar({name="workspace_x_max", description="Maximum workspace x [m]", default=0.85, required=false}),
    reqs.params.scalar({name="workspace_y_min", description="Minimum workspace y [m]", default=-0.80, required=false}),
    reqs.params.scalar({name="workspace_y_max", description="Maximum workspace y [m]", default=0.80, required=false}),
    reqs.params.scalar({name="workspace_z_min", description="Minimum workspace z [m]", default=0.05, required=false}),
    reqs.params.scalar({name="workspace_z_max", description="Maximum workspace z [m]", default=0.85, required=false})
})

linear_scale = constant(param.get("linear_scale"))
rtube_start = constant(param.get("rtube_start"))
rtube_end = constant(param.get("rtube_end"))
tube_shrink_start = constant(param.get("tube_shrink_start"))
auto_s_speed = constant(param.get("auto_s_speed"))

cartesian_damping_weight = constant(param.get("cartesian_damping_weight"))
path_projection_base_weight = constant(param.get("path_projection_base_weight"))
orientation_weight = constant(param.get("orientation_weight"))
workspace_weight = constant(param.get("workspace_weight"))

workspace_x_min = constant(param.get("workspace_x_min"))
workspace_x_max = constant(param.get("workspace_x_max"))
workspace_y_min = constant(param.get("workspace_y_min"))
workspace_y_max = constant(param.get("workspace_y_max"))
workspace_z_min = constant(param.get("workspace_z_min"))
workspace_z_max = constant(param.get("workspace_z_max"))


robot = reqs.robot_model({
    param.get("task_frame")
})
task_frame = robot.getFrame(param.get("task_frame"))

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
    target_lower = workspace_x_min,
    weight = workspace_weight,
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_x_max",
    expr = coord_x(tcp_pos),
    target_upper = workspace_x_max,
    weight = workspace_weight,
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_y_min",
    expr = coord_y(tcp_pos),
    target_lower = workspace_y_min,
    weight = workspace_weight,
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_y_max",
    expr = coord_y(tcp_pos),
    target_upper = workspace_y_max,
    weight = workspace_weight,
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_z_min",
    expr = coord_z(tcp_pos),
    target_lower = workspace_z_min,
    weight = workspace_weight,
    priority = 1,
    K = constant(4.0)
}

Constraint{
    context = ctx,
    name = "safe_z_max",
    expr = coord_z(tcp_pos),
    target_upper = workspace_z_max,
    weight = workspace_weight,
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

tube_alpha = conditional(s - tube_shrink_start, s - tube_shrink_start, constant(0.0))
tube_alpha = tube_alpha / (constant(1.0) - tube_shrink_start)
tube_alpha = conditional(tube_alpha - constant(1.0), constant(1.0), tube_alpha)

rtube = rtube_start + tube_alpha * (rtube_end - rtube_start)

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
    weight  = path_projection_base_weight + w_rvgf + w_auto_follow,
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
    weight  = orientation_weight,
    priority= 2
}

-- Skill version finish monitor:
-- exits only when automatic reaching is active and s reaches the goal.
auto_active_for_finish = conditional(w_auto_progress - constant(0.001), constant(1.0), constant(0.0))
finish_error_for_skill = (constant(1.0) - s) + (constant(1.0) - auto_active_for_finish) * constant(10.0)

Monitor{
    context = ctx,
    name = "finish_after_auto_reaches_goal",
    lower = 0.005,
    actionname = "exit",
    expr = finish_error_for_skill
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
ctx:setOutputExpression("target_x", coord_x(active_target))
ctx:setOutputExpression("target_y", coord_y(active_target))
ctx:setOutputExpression("target_z", coord_z(active_target))

ctx:setOutputExpression("distance_to_path", distance_to_path)
ctx:setOutputExpression("rtube", rtube)
ctx:setOutputExpression("G2_from_weights", G2_from_weights)
ctx:setOutputExpression("w_user_velocity", w_user_velocity) -- User velocity weight, i.e. how much joystick input affects motion. W_7 in Table I.

ctx:setOutputExpression("w_rvgf", w_rvgf)                   -- RVGF tube constraint weight, i.e. how much the RVGF tube constraint affects motion. W_11 in Table I.
ctx:setOutputExpression("w_s_damping", w_s_damping)         -- S damping weight, i.e. how much the s damping constraint affects motion. W_12 in Table I.
ctx:setOutputExpression("w_auto_progress", w_auto_progress)
ctx:setOutputExpression("w_auto_follow", w_auto_follow)

ctx:setOutputExpression("tcp_x", coord_x(tcp_pos))
ctx:setOutputExpression("tcp_y", coord_y(tcp_pos))
ctx:setOutputExpression("tcp_z", coord_z(tcp_pos))

ctx:setOutputExpression("start_x", coord_x(start_pos))
ctx:setOutputExpression("start_y", coord_y(start_pos))
ctx:setOutputExpression("start_z", coord_z(start_pos))

ctx:setOutputExpression("pos_err_x", coord_x(pos_err))
ctx:setOutputExpression("pos_err_y", coord_y(pos_err))
ctx:setOutputExpression("pos_err_z", coord_z(pos_err))
