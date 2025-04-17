
abstract type PolylineSegment end

function perp(x)
    [-x[2], x[1]]
end

struct InitialRay <: PolylineSegment
    point::SVector{2, Float64}
    tangent::SVector{2, Float64}
    normal::SVector{2, Float64}
    function InitialRay(point, next_point)
        tangent = next_point - point
        tangent ./= norm(tangent)
        tangent = normalize(next_point-point)
        normal = perp(tangent)
        new(point, tangent, normal)
    end
end
struct TerminalRay <: PolylineSegment
    point::SVector{2, Float64}
    tangent::SVector{2, Float64}
    normal::SVector{2, Float64}
    function TerminalRay(point, prev_point)
        tangent = normalize(point - prev_point)
        tangent = point - prev_point
        tangent ./= norm(tangent)
        normal = perp(tangent)
        new(point, tangent, normal)
    end
end

struct StandardSegment <: PolylineSegment
    p1::SVector{2, Float64}
    p2::SVector{2, Float64}
    tangent::SVector{2, Float64}
    normal::SVector{2, Float64}
    function StandardSegment(p1, p2)
        tangent = p2 - p1
        tangent ./= norm(tangent)
        normal = perp(tangent)
        new(p1, p2, tangent, normal)
    end
end

struct Polyline
    segments::Vector{PolylineSegment}
    function Polyline(points)
        segments = Vector{PolylineSegment}()
        N = length(points)
        @assert N ≥ 2
        initial_ray = InitialRay(points[1], points[2]) 
        push!(segments, initial_ray)
        for i = 1:(N-1)
            seg = StandardSegment(points[i], points[i+1])
            push!(segments, seg)
        end
        terminal_ray = TerminalRay(points[end], points[end-1])
        push!(segments, terminal_ray)
        new(segments)
    end
    function Polyline(points...)
        Polyline(points)
    end
end

function signed_distance(polyline::Polyline, point)
    min_signed = Inf
    for segment in polyline.segments
        if segment isa StandardSegment
            p1 = segment.p1
            p2 = segment.p2
            pl_direction = p2-p1
            vec_to_p1 = point-p1 
            projection_on_polyline = dot(vec_to_p1,pl_direction)/dot(pl_direction,pl_direction)
            if projection_on_polyline < 0 || projection_on_polyline > 1
                continue
            end
                closest_pt_on_polyline = p1 + pl_direction * projection_on_polyline
        elseif segment isa InitialRay 
            vec_to_point = point - segment.point
            proj = dot(vec_to_point, segment.tangent)

            if proj >= 0
                closest_pt_on_polyline = segment.point
            else
                closest_pt_on_polyline = segment.point + proj * segment.tangent
            end
        elseif segment isa TerminalRay 
            vec_to_point = point - segment.point
            proj = dot(vec_to_point, segment.tangent)

            if proj <= 0 
                closest_pt_on_polyline = segment.point
            else 
                closest_pt_on_polyline = segment.point + proj * segment.tangent
            end
        end
        dist = norm(point - closest_pt_on_polyline)
        signed_dist = dist * sign(dot(segment.normal, point - closest_pt_on_polyline))
        if abs(signed_dist) < abs(min_signed)
            min_signed = signed_dist
        end
    end
    return min_signed
end



function routing(map,ego_road_segment_id,target_road_segment_id)
    # Dijkstras on an edge list for no assume all road segments are the same distance. 
    # Later we will compute edge weights using lane bounderies divided by speed limit.
    dist = Dict(k => Inf for k in keys(map))
    prev = Dict{Int, Union{Nothing, Int}}()
    for k in keys(map)
        prev[k] = nothing
    end

    Q = PriorityQueue{Int,Float64}()

    dist[ego_road_segment_id] = 0


    
    enqueue!(Q, ego_road_segment_id, 0.0)

    counter = 1
    while !isempty(Q)
        counter+=1

        u = dequeue!(Q) # dequeue seg id and dist from ego_road_segment_id

        if u == target_road_segment_id
            break
        end

        for v in map[u].children 
            alt = dist[u] + 1.0 # the distance here is assumed to be 1 but in reality it should probablyt be a time (Roadsegment_dist * 1/(speed_limit))
            if alt < dist[v]
                dist[v] = alt
                prev[v] = u
                enqueue!(Q,v,alt)
            end

        end
    end
    @info "finished searching"



    path = []
    u = target_road_segment_id
    while u !== nothing
        push!(path,u)
        u = prev[u]
    end 

    reverse!(path)
    return path
end


# mutable struct RoadSegment
#     id::Int
#     lane_boundaries::Vector{LaneBoundary}
#     lane_types::Vector{LaneTypes}
#     speed_limit::Float64
#     children::Vector{Int}
# end

# each loading zone is broken up into three parts.
LOADING_ZONE_KEYS =  [56,
54,
52,
92,
82,
77,
25,
27,
42,
79,
81,
29,
94,
38,
78,
80,
96,
40]

STOP_SIGNS = [13,
86,
76,
61,
101,
47,
83,
14]


function interpolate_lane_boundary(boundary::VehicleSim.LaneBoundary; num_points::Int = 20)
    a, b = boundary.pt_a, boundary.pt_b
    κ = boundary.curvature

    if abs(κ) < 1e-6
        return [a, b]
    end

    # Arc interpolation
    chord = b - a
    chord_length = norm(chord)
    mid_pt = 0.5 .* (a + b)
    dir_vec = normalize(chord)
    normal_vec = SVector(-dir_vec[2], dir_vec[1])

    R = 1 / κ
    sagitta_height = sqrt(R^2 - (chord_length / 2)^2)
    center = mid_pt + sign(κ) * sagitta_height * normal_vec

    θ_start = atan(a[2] - center[2], a[1] - center[1])
    θ_end   = atan(b[2] - center[2], b[1] - center[1])

    # Ensure correct direction
    if κ > 0 && θ_end < θ_start
        θ_end += 2π
    elseif κ < 0 && θ_end > θ_start
        θ_end -= 2π
    end

    thetas = range(θ_start, θ_end; length=num_points)

    return [[center[1] + R*cos(θ), center[2] + R*sin(θ)] for θ in thetas]
    # return [center .+ R .* SVector(cos(θ), sin(θ)) for θ in thetas]
end

function get_center_line(seg; num_points=20)
    left_bndry, right_bndry = seg.lane_boundaries

    if abs(left_bndry.curvature) < 1e-6 && abs(right_bndry.curvature) < 1e-6
        # start_center = 0.5 .* (left_bndry.pt_a + right_bndry.pt_a)
        # end_center   = 0.5 .* (left_bndry.pt_b + right_bndry.pt_b)

        start_center = [(left_bndry.pt_a[1] + right_bndry.pt_a[1]) / 2,
                (left_bndry.pt_a[2] + right_bndry.pt_a[2]) / 2]

        end_center = [(left_bndry.pt_b[1] + right_bndry.pt_b[1]) / 2,
                    (left_bndry.pt_b[2] + right_bndry.pt_b[2]) / 2]
        return [start_center, end_center]
    end

    # Curved case
    pts_left  = interpolate_lane_boundary(left_bndry; num_points)
    pts_right = interpolate_lane_boundary(right_bndry; num_points)

    # Match lengths
    min_len = min(length(pts_left), length(pts_right))
    # centerline = [(pts_left[i] + pts_right[i]) .* 0.5 for i in 1:min_len]

    centerline = [[(pts_left[i][1] + pts_right[i][1]) / 2,
               (pts_left[i][2] + pts_right[i][2]) / 2] for i in 1:min_len]

    return centerline
end


# function get_way_points_from_path(path,map)
#     waypoints = []
#     for id in path
#         centerline = get_center_line(map[id],num_points=3)
#         append!(waypoints,centerline)
#     end

#     return waypoints
# end


function get_way_points_from_path(path, map)
    waypoints = Vector{Vector{Float64}}()
    road_id = Vector{Int}()
    first = true
    for id in path
        centerline = get_center_line(map[id], num_points=4)
        if first
            append!(waypoints, centerline)
            n = length(centerline)
            ids = [id for i in 1:n]
            append!(road_id,ids)
            first = false
        else
            # Drop the first point to avoid overlap with previous segment’s last point
            append!(waypoints, centerline[2:end])
            n = length(centerline[2:end])
            ids = [id for i in 1:n]
            append!(road_id,ids)
        end
    end
    return waypoints,road_id
end

function get_polyline(waypoints)
    polyline = Polyline(waypoints)
    return polyline

end

function get_path(map,ego_id,target_segment_id)
    path_ids = routing(map,ego_id,target_segment_id)
    waypoints, road_ids = get_way_points_from_path(path_ids, map)
    pl = DriveGoodAVStack.Polyline(waypoints)
    return (pl,road_ids)
end


