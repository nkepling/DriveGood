function compute_control(path::Vector{SVector{2,Float64}}, pos::SVector{2,Float64}, heading::Float64;
        lookahead=5.0, speed=2.0, wheelbase=2.5)

    direction = SVector(cos(heading), sin(heading))
    goal = find_lookahead_point(path, pos, lookahead, direction)

    if goal === nothing
        return (0.0, 0.0, false)
    end

    rel = goal - pos
    x_local = dot(rel, direction)
    y_local = cross(direction, rel)

    if x_local ≤ 0
        return (0.0, 0.0, false)
    end

    curvature = (2 * y_local) / (lookahead^2)
    steering = atan(wheelbase * curvature)
    return (steering, speed, true)
end
