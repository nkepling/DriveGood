using Sockets
using Serialization
using VehicleSim
using LinearAlgebra
using DataStructures

include("decision/decision_making.jl")

function process_gt(gt_channel, shutdown_channel, localization_state_channel, perception_state_channel, routing_channel, target_segment)
    @info "[GT] process_gt() started"
    while true
        fetch(shutdown_channel) && break

        fresh_gt_meas = []
        while isready(gt_channel)
            push!(fresh_gt_meas, take!(gt_channel))
        end

        if isempty(fresh_gt_meas)
            sleep(0.01)
            continue
        end

        latest = fresh_gt_meas[end]

        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, latest)

        if isready(routing_channel)
            take!(routing_channel)  # clear old route if any
        end
        
        current_pos = SVector{2, Float64}(latest.position[1], latest.position[2])
        route = build_routing_path(VehicleSim.city_map(), current_pos,  target_segment)
        put!(routing_channel, route)
        @info "[Routing] Built GT-based route with $(length(route)) points"
        

        dummy_obstacles = []  # Or put something fake if you want to test avoidance
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, dummy_obstacles)

        @info "[GT] Updated localization and perception"
    end
end

const STOP_SIGNS = Set([13, 86, 76, 61, 101, 47, 83, 14])
const LOADING_ZONE_KEYS = Set([56,54,52,92,82,77,25,27,42,79,81,29,94,38,78,80,96,40])

function routing(map, start_id, goal_id)
    dist = Dict(k => Inf for k in keys(map))
    prev = Dict{Int, Union{Nothing, Int}}(k => nothing for k in keys(map))
    Q = PriorityQueue{Int, Float64}()
    dist[start_id] = 0.0
    enqueue!(Q, start_id, 0.0)

    while !isempty(Q)
        u = dequeue!(Q)
        if u == goal_id
            break
        end
        for v in map[u].children
            alt = dist[u] + 1.0
            if alt < dist[v]
                dist[v] = alt
                prev[v] = u
                enqueue!(Q, v, alt)
            end
        end
    end

    path = []
    u = goal_id
    while u !== nothing
        push!(path, u)
        u = prev[u]
    end
    reverse!(path)
    return path
end

function get_center_line(seg; num_points=4)
    a, b = seg.lane_boundaries
    pt1 = [(a.pt_a[1] + b.pt_a[1]) / 2, (a.pt_a[2] + b.pt_a[2]) / 2]
    pt2 = [(a.pt_b[1] + b.pt_b[1]) / 2, (a.pt_b[2] + b.pt_b[2]) / 2]
    return [pt1, pt2]
end

function get_way_points_from_path(path, map)
    waypoints = []
    road_ids = []
    first = true
    for id in path
        pts = get_center_line(map[id])
        if first
            append!(waypoints, pts)
            append!(road_ids, fill(id, length(pts)))
            first = false
        else
            append!(waypoints, pts[2:end])
            append!(road_ids, fill(id, length(pts[2:end])))
        end
    end
    return waypoints, road_ids
end

function get_closest_road_segment(pos::SVector{2, Float64}, map)
    min_dist = Inf
    closest_id = -1
    for (id, segment) in map
        pts = get_center_line(segment)
        for pt in pts
            pt_vec = SVector{2, Float64}(pt...)
            dist = norm(pt_vec - pos)
            if dist < min_dist
                min_dist = dist
                closest_id = id
            end
        end
    end
    return closest_id
end


function build_routing_path(map, current_pos, target_segment)
    start_seg = get_closest_road_segment(current_pos, map)
    path_ids = routing(map, start_seg, target_segment)
    waypoints, road_ids = get_way_points_from_path(path_ids, map)

    route = []
    for (i, pt) in enumerate(waypoints)
        seg_id = road_ids[min(i, end)]
        stop = seg_id in STOP_SIGNS
        pullout = seg_id in LOADING_ZONE_KEYS
        push!(route, ((pt[1], pt[2]), stop, pullout))
    end
    return route
end

function my_client(host::IPAddr = IPv4(0), port = 4444)
    @info "[Client] Connecting to VehicleSim at $(host):$(port)"
    socket = Sockets.connect(host, port)
    @info "[Client] Connected to socket"

    map_segments = VehicleSim.city_map()
    map!
    @info "[Client] Map loaded"

    msg = deserialize(socket)
    @info "[Client] Received initial message from sim"
    while msg isa String
        @info "Skipping string message: $msg"
        msg = deserialize(socket)
    end

    ego_id = msg.vehicle_id
    target_segment = msg.target_segment

    
    @info "Connected to VehicleSim. Vehicle ID: $ego_id"

    # Channels
    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{Any}(1)
    perception_state_channel = Channel{Any}(1)
    routing_channel = Channel{Any}(1)
    shutdown_channel = Channel{Bool}(1)
    put!(shutdown_channel, false)

    # Measurement listener
    errormonitor(@async begin
        while true
            sleep(0.001)
            local msg
            received = false

            while true
                @async eof(socket)
                if bytesavailable(socket) > 0
                    msg = deserialize(socket)
                    received = true
                else
                    break
                end
            end

            !received && continue

            for meas in msg.measurements
                if meas isa GPSMeasurement
                    put!(gps_channel, meas)
                elseif meas isa IMUMeasurement
                    put!(imu_channel, meas)
                elseif meas isa CameraMeasurement
                    put!(cam_channel, meas)
                elseif meas isa GroundTruthMeasurement
                    put!(gt_channel, meas)
                end
            end





        end
    end)

    errormonitor(@async process_gt(
    gt_channel,
    shutdown_channel,
    localization_state_channel,
    perception_state_channel,
    routing_channel,
    target_segment
))

    errormonitor(@async run_planner(
        localization_state_channel,
        perception_state_channel,
        routing_channel,
        socket,
        shutdown_channel
    ))

    
    
    

end

# 🚗 Run client
my_client(ip"10.74.101.131", 4444)  # Replace with your IP if needed 


