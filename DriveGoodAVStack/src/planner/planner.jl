module Planner

export plan_motion

using StaticArrays
using ..PurePursuit
using ..GeometryUtils
using LinearAlgebra


function wrap_angle(θ)
    while θ > π
        θ -= 2π
    end
    while θ < -π
        θ += 2π
    end
    return θ
end

function plan_motion(loc, obstacles, path)
    pos = SVector(loc.position[1], loc.position[2])
    heading = loc.heading
    velocity = norm(loc.velocity)

    for ob in obstacles
        dx = ob[1] - pos[1]
        dy = ob[2] - pos[2]
        dist = hypot(dx, dy)
        angle_to_ob = atan(dy, dx)
        rel_angle = abs(wrap_angle(angle_to_ob - heading))
        if dist < 6.0 && rel_angle < π/4
            return (0.0, 0.0, true)
        end
    end

    #= 
safe_path = try_shifted_path(path, pos, heading, obstacles)
steering, throttle, ok = PurePursuit.compute_control(safe_path, pos, heading)
=#

    steering, throttle, ok = PurePursuit.compute_control(path, pos, heading)
    if !ok
        return (0.0, 0.0, true)
    end

    return (steering, throttle, true)
end


#=
"""
    try_shifted_path(original_path, pos, heading, obstacles)

If there's an obstacle ahead, try shifting the path laterally
and check if it's clear. Returns a safe path (shifted or original).
"""
function try_shifted_path(original_path, pos, heading, obstacles; shift_distance=3.0)
    shifted = [(pt[1], pt[2] + shift_distance) for pt in original_path]
    shifted_vec = [SVector(x, y) for (x, y) in shifted]

    facing = SVector(cos(heading), sin(heading))

    function path_is_clear(path_pts)
        for ob in obstacles
            for pt in path_pts
                rel = SVector(ob[1] - pt[1], ob[2] - pt[2])
                dist = norm(rel)
                angle = atan(rel[2], rel[1])
                if dist < 5.0 && abs(angle - heading) < π/4
                    return false
                end
            end
        end
        return true
    end

    if path_is_clear(shifted_vec)
        @info "[Planner] Lane shift SUCCESS: using shifted path"
        return shifted_vec
    else
        @info "[Planner] Lane shift blocked: using original path"
        return original_path
    end
end
=#

end
