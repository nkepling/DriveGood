using LinearAlgebra
using StaticArrays
using VehicleSim

# ──────────────── Utility ────────────────
function wrap_angle(θ)
    while θ > π
        θ -= 2π
    end
    while θ < -π
        θ += 2π
    end
    return θ
end


# ──────────────── Pure Pursuit ────────────────
function compute_control(path, pos::Tuple, heading::Float64;
                         lookahead_distance=8.0, velocity=2.5, wheelbase=2.5)

    position = SVector(pos[1], pos[2])
    facing = SVector(cos(heading), sin(heading))

    for (pt, _, _) in path
        pt_vec = SVector(pt[1], pt[2])
        to_point = pt_vec - position
        dist = norm(to_point)
        angle_to_point = atan(to_point[2], to_point[1])
        angle_diff = wrap_angle(angle_to_point - heading)

        @info "[PP] Checking point: $pt | dist = $(round(dist, digits=2)), angle_diff = $(round(angle_diff, digits=2))"

        if dist > lookahead_distance && abs(angle_diff) < π 
            dx, dy = pt_vec - position
            lx = cos(heading) * dx + sin(heading) * dy
            ly = -sin(heading) * dx + cos(heading) * dy

            if lx <= 0
                @warn "[PP] Lookahead point is behind vehicle — skipping"
                continue
            end

            curvature = (2 * ly) / (lookahead_distance^2)
            steer = atan(wheelbase * curvature)

            @info "[PP] Lookahead success — Steer=$(round(steer, digits=2))"
            return (steer, velocity, true)
        end
    end

    @warn "[PP] No valid lookahead point found — stopping"
    return (0.0, 0.0, false)
end


# ──────────────── Motion Planner ────────────────
function plan_motion(loc, obstacles, path, state, stop_timer, stopped_at_time)
    pos = Tuple(loc.position[1:2])
    heading = loc.heading

    # 🚧 Obstacle avoidance
    for ob in obstacles
        dx = ob[1] - pos[1]
        dy = ob[2] - pos[2]
        dist = hypot(dx, dy)
        angle = wrap_angle(atan(dy, dx) - heading)
        if dist < 6.0 && abs(angle) < π/4
            @info "[Planner] Obstacle ahead – braking | dist=$(round(dist, digits=2)) angle=$(round(angle, digits=2))"
            return :STOPPED, VehicleCommand(0.0, 0.0, true), stopped_at_time, stop_timer
        end
    end

    # 🚦 Find nearest waypoint
    closest_i = argmin([hypot(pt[1][1] - pos[1], pt[1][2] - pos[2]) for pt in path])
    wp, stop, pullout = path[closest_i]

    dist_to_wp = hypot(wp[1] - pos[1], wp[2] - pos[2])
    @info "[Planner] Closest WP=$closest_i at dist=$(round(dist_to_wp, digits=2)) | stop=$stop pullout=$pullout"

    # 🛑 Stop sign logic
    if stop && dist_to_wp < 5.0 && state != :WAITING
        @info "[Planner] Stop sign ahead – stopping"
        return :WAITING, VehicleCommand(0.0, 0.0, true), time(), stop_timer
    end

    if state == :WAITING
        elapsed = time() - stopped_at_time
        if elapsed < 2.0
            @info "[Planner] Waiting at stop sign... elapsed=$(round(elapsed, digits=2))"
            return :WAITING, VehicleCommand(0.0, 0.0, true), stopped_at_time, stop_timer
        else
            @info "[Planner] Done waiting – resuming"
            state = :DRIVING
        end
    end

    # 🅿️ Pullout zone
    if pullout && dist_to_wp < 3.0
        @info "[Planner] Pullout zone reached – parking"
        return :PARKED, VehicleCommand(0.0, 0.0, false), stopped_at_time, stop_timer
    end

    # 🧭 Compute control
    steer, throttle, ok = compute_control(path, pos, heading)
    if !ok
        return state, VehicleCommand(0.0, 0.0, true), stopped_at_time, stop_timer
    end

    return state, VehicleCommand(steer, throttle, true), stopped_at_time, stop_timer
end

# ──────────────── Planner Loop ────────────────
function run_planner(loc_channel, perception_channel, path_channel, socket, shutdown_channel)
    @info "Planner is active..."
    state = :DRIVING
    stop_timer = 0.0
    stopped_at_time = 0.0

    while true
        fetch(shutdown_channel) && break

        raw = fetch(loc_channel)
        heading = VehicleSim.extract_yaw_from_quaternion(raw.orientation)



        loc = (
            position = raw.position,
            heading = heading,
            velocity = raw.velocity
        )

        obs = fetch(perception_channel)
        path = fetch(path_channel)

        @info "[Planner] Localization: $(loc.position), Heading: $(loc.heading)"
        @info "[Planner] Obstacles: $obs"
        @info "[Planner] Path: $path"

        state, cmd, stopped_at_time, stop_timer = plan_motion(loc, obs, path, state, stop_timer, stopped_at_time)

        cmd_tuple = (cmd.steering_angle, cmd.velocity, cmd.controlled)

@info "[Planner] sending tuple: $(cmd_tuple) of type $(typeof(cmd_tuple))"
#serialize(socket, cmd_tuple)


       # Inside run_planner loop

@info "[Planner] Command: Steer=$(cmd.steering_angle), Velocity=$(cmd.velocity), Controlled=$(cmd.controlled)"
@info "[Debug] VehicleCommand struct: $(cmd)"
@info "[Debug] Type check: $(typeof(cmd)), Fields: $(fieldnames(typeof(cmd)))"

try
    # 🚨 This is the important change — send a tuple!
    serialize(socket, (cmd.steering_angle, cmd.velocity, cmd.controlled))
    @info "[Socket] Sent tuple to VehicleSim: ($(cmd.steering_angle), $(cmd.velocity), $(cmd.controlled))"
catch e
    @error "[Socket] Failed to send command to VehicleSim: $e"
end

sleep(0.05)

    end
end 

