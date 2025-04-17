

using StaticArrays
using LinearAlgebra
using VehicleSim
using Serialization


function wrap_angle(θ)
    return mod(θ + π, 2π) - π
end

#=function compute_control(path, pos::Tuple, heading::Float64;
    lookahead_distance=3.0, max_lookahead=15.0, velocity=2.5, wheelbase=2.5)

    position = SVector(pos...)
    facing = SVector(cos(heading), sin(heading))

    best_dot = -Inf
    best_point = nothing

    for (pt, _, _) in path
        pt_vec = SVector(pt...)
        to_point = pt_vec - position
        dist = norm(to_point)

        if dist < lookahead_distance || dist > max_lookahead
            continue
        end

        sim = dot(to_point, facing)
        if sim <= 0
            continue
        end

        if sim > best_dot
            best_dot = sim
            best_point = pt_vec
            @info "[PP] Candidate: $pt_vec | dist=$(round(dist, digits=2)) | sim=$(round(sim, digits=2))"
        end
    end

    if best_point === nothing
        @warn "[PP] No valid lookahead point found — stopping"
        return (0.0, 0.0, false)
    end

    dx, dy = best_point - position
    alpha = wrap_angle(atan(dy, dx) - heading)
    steer = atan(2 * wheelbase * sin(alpha) / lookahead_distance)
    v = velocity * exp(-abs(steer))

    @info "[PP] Chosen point: $best_point, Steering=$(round(steer, digits=3)), Velocity=$(round(v, digits=2))"
    return (steer, v, true)
end
=#

function compute_control(path, pos::Tuple, heading::Float64;
    current_index::Int=1, lookahead_points::Int=5,
    lookahead_distance=8.0, velocity=2.5, wheelbase=2.5)

    position = SVector(pos...)
    facing = SVector(cos(heading), sin(heading))

    target_index = min(current_index + lookahead_points, length(path))
    target_wp = path[target_index][1]
    target_vec = SVector(target_wp...)

    dx, dy = target_vec - position
    alpha = wrap_angle(atan(dy, dx) - heading)
    
    steer = atan(2 * wheelbase * sin(alpha) / lookahead_distance)
    v = velocity * exp(-abs(steer))  # optional: slow down on sharp turns

    @info "[PP] Target WP index=$target_index | Steering=$(round(steer, digits=3)), Velocity=$(round(v, digits=2))"
    return (steer, v, true)
end



function plan_motion(loc, obstacles, path, state, stop_timer, stopped_at_time, current_wp_index)

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
    # Only look at remaining waypoints
remaining_path = path[current_wp_index:end]
dists = [hypot(pt[1][1] - pos[1], pt[1][2] - pos[2]) for pt in remaining_path]
closest_rel_idx = argmin(dists)
closest_i = current_wp_index - 1 + closest_rel_idx

    wp, stop, pullout = path[closest_i]
    @info "[PP] Targeting WP[$closest_i] = $(wp)"


    dist_to_wp = hypot(wp[1] - pos[1], wp[2] - pos[2])
    if dist_to_wp < 2.0 && current_wp_index < length(path)
        current_wp_index += 1
        @info "[Planner] Advancing to WP[$current_wp_index]"
    end
    
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

    @info "[Debug] Position: $(pos), Heading: $(heading)"
@info "[Debug] Closest WP: $(wp)"
@info "[Debug] Stop = $(stop), Pullout = $(pullout)"


    # 🧭 Compute control
    

    steer, throttle, ok = compute_control(path, pos, heading; current_index=current_wp_index)



if !ok
    return state, VehicleCommand(0.0, 0.0, true), stopped_at_time, stop_timer
end

@info "[Control] Steer=$(steer), Velocity=$(throttle)"



return state, VehicleCommand(steer, throttle, true), stopped_at_time, stop_timer, current_wp_index

end

# ──────────────── Planner Loop ────────────────
function run_planner(loc_channel, perception_channel, path_channel, socket, shutdown_channel)
    @info "Planner is active..."
    cached_path = nothing  # cache the route after first read
    current_wp_index = 1


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
        if isnothing(cached_path)
            @info "[Planner] Waiting for route to arrive..."
            while !isready(path_channel)
                sleep(0.05)
            end
            cached_path = take!(path_channel)
            @info "[Planner] Got route with $(length(cached_path)) points"
        end
        
        
        # From here on, use the cached path
        path = cached_path
        
        
        @info "[Planner] Path length: $(length(path))"


        @info "[Planner] Localization: $(loc.position), Heading: $(loc.heading)"
        @info "[Planner] Obstacles: $obs"
        @info "[Planner] Path: $path"

        state, cmd, stopped_at_time, stop_timer, current_wp_index = plan_motion(loc, obs, path, state, stop_timer, stopped_at_time, current_wp_index)

       #cmd = VehicleCommand(0.0, 5.0, true)  # steer = 0, velocity = 5
#@info "[FORCE TEST] Forcing vehicle to move straight"


        cmd_tuple = (cmd.steering_angle, cmd.velocity, cmd.controlled)

@info "[Planner] sending tuple: $(cmd_tuple) of type $(typeof(cmd_tuple))"
#serialize(socket, cmd_tuple)


       # Inside run_planner loop

@info "[Planner] Command: Steer=$(cmd.steering_angle), Velocity=$(cmd.velocity), Controlled=$(cmd.controlled)"
@info "[Debug] VehicleCommand struct: $(cmd)"
@info "[Debug] Type check: $(typeof(cmd)), Fields: $(fieldnames(typeof(cmd)))"

try
    # 🚨 This is the important change — send a tuple!
    @info "[Planner] About to send: steer=$(cmd.steering_angle), velocity=$(cmd.velocity), controlled=$(cmd.controlled)"

    serialize(socket, (cmd.steering_angle, cmd.velocity, cmd.controlled))
    @info "[Socket] Sent tuple to VehicleSim: ($(cmd.steering_angle), $(cmd.velocity), $(cmd.controlled))"
catch e
    @error "[Socket] Failed to send command to VehicleSim: $e"
end

sleep(0.05)

    end
end