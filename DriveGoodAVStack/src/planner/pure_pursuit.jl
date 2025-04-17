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

function find_lookahead_point(pl::Polyline,
                              pos::SVector{2,Float64},
                              lookahead::Float64)
    best_pt = nothing
    best_prog = -Inf
    cum_dist = 0.0

    for seg in pl.segments
        if seg isa StandardSegment
            p1, p2 = seg.p1, seg.p2
            d = p2 - p1
            a = dot(d,d)
            b = 2 * dot(p1 .- pos, d)
            c = dot(p1 .- pos, p1 .- pos) - lookahead^2
            disc = b^2 - 4a*c
            if disc ≥ 0
                sqrt_disc = sqrt(disc)
                for t in ((-b + sqrt_disc)/(2a), (-b - sqrt_disc)/(2a))
                    if 0.0 ≤ t ≤ 1.0
                        pt = p1 + t*d
                        prog = cum_dist + norm(d)*t
                        if prog > best_prog
                            best_prog, best_pt = prog, pt
                        end
                    end
                end
            end
            cum_dist += norm(d)
        end
    end

    return best_pt
end

# normalize into (–π,π]
wrap_angle(θ) = mod(θ + π, 2π) - π

function calculate_angle(pl::Polyline,
    pos::SVector{2,Float64},
    heading::Float64;
    lookahead=5.0,
    speed=2.0)

# 1) find lookahead point
    goal = find_lookahead_point(pl, pos, lookahead)
    goal === nothing && return (0.0, 0.0, false)

# 2) vector from vehicle to goal
    Δ = goal .- pos

# 3) absolute bearing to goal
    bearing_to_goal = atan2(Δ[2], Δ[1])

# 4) signed angle error = goal_bearing minus current heading
    angle_error = wrap_angle(bearing_to_goal - heading)

# if the goal is effectively behind us, bail out
# (optional — you can remove if you still want a “behind” bearing)
    x_f = dot(SVector(cos(heading), sin(heading)), Δ)
    if x_f ≤ 0
        return (0.0, 0.0, false)
    end

return (angle_error)
end



# function control(state_ch, control_ch, stop_ch, path)
#     pure_pursuit(state_ch, control_ch, stop_ch, path)
#     # or try dummy_controller(...) if you like
# end

# #makes angle between -pi, pi
# function wrap_angle(θ)
#     return mod(θ + π, 2π) - π
# end
# function pure_pursuit(localization, control_ch, stop_ch, path; ls = 0.5, L =3)

#     while true
#         sleep(0.1)
#         fetch(stop_ch) && return  # stop if signaled

#         # Fetch current state: x = (p1, p2, θ, v)
#         x = fetch(state_ch)
#         p1, p2, θ, v = x
#         lookahead = v * ls
#         best_progress = -Inf
#         target_pt = nothing
#         cumulative_dist = 0.0
#         center = SVector(p1, p2)  
#         for segment in path.segments
#             # loop ensures that we find the farthest along the polyline point to aim to. 
#             if segment isa StandardSegment
#                 s1 = segment.p1
#                 s2 = segment.p2
#                 pl_direction = s2 - s1  
#                 pl_to_center = s1 - center

#                 # solve |s1 + t*pl_direction - center|^2 = lookahead^2.
#                 # set up quadratic by expanding the left side. a^2 + 2ab + c^2
#                 # subtrack lookahead^2 to move everything to one side
#                 # quad formula.
#                 a_val = dot(pl_direction, pl_direction)
#                 b_val = 2 * dot(pl_to_center, pl_direction)
#                 c_val = dot(pl_to_center, pl_to_center) - lookahead^2
#                 disc = b_val^2 - 4 * a_val * c_val 

#                 if disc < 0
#                     #imaginary answer
#                 else
#                     #ensure t is from 0 to 1, meaning point is on the seg
#                     sqrt_disc = sqrt(disc)
#                     for t in ((-b_val + sqrt_disc) / (2 * a_val),
#                               (-b_val - sqrt_disc) / (2 * a_val))
#                         if 0.0 <= t <= 1.0
#                             # point is on this segment.
#                             pt = s1 + t * pl_direction
#                             progress = cumulative_dist + t * norm(pl_direction)
#                             if progress > best_progress
#                                 best_progress = progress
#                                 target_pt = pt
#                             end
#                         end
#                     end
#                 end
#                 cumulative_dist += norm(pl_direction)
#             end
#         end

#         if target_pt === nothing
#             δ = 0.0
#         else
#         #math from lec 11
#         dist_x = target_pt[1] - p1
#         dist_y = target_pt[2] - p2
#         heading_to_target = atan(dist_y, dist_x)
#         alpha = wrap_angle(heading_to_target - θ)
#         gamma = atan(2 * L * sin(alpha) / lookahead)
#         a = 0
        
#         #slow on corners, speed up on straights. 
#         if abs(gamma) > 0.100 
#             a = -0.75
#         elseif abs(gamma) < 0.100 && v < 5
#             a = 1
#         end
        
#         take!(control_ch)  # clear old control
#         put!(control_ch, @SVector [gamma, a])
#     end
# end
# end


