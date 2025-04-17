# Normalize angle to [-π, π]
function normalize_angle(angle::Float64)
    return mod(angle + π, 2π) - π
end

# Check if waypoint is ahead of vehicle
function is_waypoint_ahead(car_x, car_y, yaw, wp_x, wp_y)
    dx = wp_x - car_x
    dy = wp_y - car_y
    heading_vec = [cos(yaw), sin(yaw)]
    wp_vec = [dx, dy]
    dot_prod = dot(heading_vec, wp_vec)
    return dot_prod > 0
end

# Stanley controller with filtering of behind-waypoints
function stanley_control(loc,
                         v,
                         waypoints;
                         k=1.0)
    deadband = 0.1  # meters
    x = loc.lat
    y = loc.long
    yaw = loc.yaw
    # Filter waypoints ahead of vehicle
    ahead_waypoints = filter(wp -> is_waypoint_ahead(x, y, yaw, wp[1], wp[2]), waypoints)
    if isempty(ahead_waypoints)
        println("Warning: No waypoints ahead — defaulting to full list.")
        ahead_waypoints = waypoints
    end

    # Extract path points
    path_x = [pt[1] for pt in ahead_waypoints]
    path_y = [pt[2] for pt in ahead_waypoints]

    # Find nearest waypoint
    dists = [hypot(px - x, py - y) for (px, py) in zip(path_x, path_y)]
    nearest_idx = argmin(dists)

    # Compute path heading (between nearest and next point)
    next_idx = min(nearest_idx + 1, length(path_x))
    dx = path_x[next_idx] - path_x[nearest_idx]
    dy = path_y[next_idx] - path_y[nearest_idx]
    path_yaw = atan(dy, dx)

    # Heading error
    heading_error = normalize_angle(path_yaw - yaw)

    # Cross-track error (signed)
    dx = path_x[nearest_idx] - x
    dy = path_y[nearest_idx] - y
    cross_track_error = dy * cos(yaw) - dx * sin(yaw)

    # if abs(cross_track_error) < deadband
    #     cross_track_error = 0.0
    # end

    # Stanley control law
    cross_track_term = atan(k * cross_track_error / (v + 1e-5))
    delta = normalize_angle(heading_error + cross_track_term)

    return delta,nearest_idx
end


function smooth_steering(δ_current, δ_target, max_rate, dt)
    Δδ = δ_target - δ_current
    Δδ_clipped = clamp(Δδ, -max_rate * dt, max_rate * dt)
    return δ_current + Δδ_clipped
end

function smooth_velocity(v_current, v_target, max_accel, dt)
    Δv = v_target - v_current
    Δv_clipped = clamp(Δv, -max_accel * dt, max_accel * dt)
    return v_current + Δv_clipped
end
