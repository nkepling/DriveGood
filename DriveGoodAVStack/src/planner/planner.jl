const STOP_DIST = 3.5
const PULLOUT_DIST = 3.0

function wrap_angle(θ)
    while θ > π
        θ -= 2π
    end
    while θ < -π
        θ += 2π
    end
    return θ
end

function plan_motion(loc, obstacles, path, state, stop_timer, stopped_at_time)
    pos = SVector(loc.position[1], loc.position[2])
    heading = loc.heading
    velocity = norm(loc.velocity)

    # --- Detect Obstacle ---
    for ob in obstacles
        dist = norm(SVector(ob[1], ob[2]) - pos)
        angle = abs(wrap_angle(atan(ob[2] - pos[2], ob[1] - pos[1]) - heading))
        if dist < 6.0 && angle < π/4
            @info "[Planner] Obstacle ahead – braking"
            return 0.0, 0.0, true, :STOPPING, stop_timer, stopped_at_time
        end
    end

    # --- Stop sign handling ---
    for (pt, is_stop, is_pullout) in path
        dist = norm(pt - pos)
        if is_stop && dist < 5.0
            if state != :WAITING
                @info "[Planner] Stop sign ahead – stopping"
                return 0.0, 0.0, true, :WAITING, 0.0, time()
            end
        end
    end

    # If waiting at stop sign
    if state == :WAITING
        stop_timer = time() - stopped_at_time
        if stop_timer < 3.0
            return 0.0, 0.0, true, :WAITING, stop_timer, stopped_at_time
        else
            @info "[Planner] Done waiting – resuming"
            state = :DRIVING
        end
    end

    # --- Pullout zone ---
    for (pt, _, is_pullout) in path
        dist = norm(pt - pos)
        if is_pullout && dist < 5.0
            @info "[Planner] Pullout zone reached – parking"
            return 0.0, 0.0, false, :PULLOUT, stop_timer, stopped_at_time
        end
    end

    # Normal tracking
    steer, throttle, ok = compute_control([p[1] for p in path], pos, heading)
    @info "this is the throttle: $throttle"
    return steer, throttle, ok, state, stop_timer, stopped_at_time
end
