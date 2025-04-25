
function process_gt(
    gt_channel::Channel{GroundTruthMeasurement},
    shutdown_channel::Channel{Bool},
    localization_state_channel::Channel{LocalizationType},
    perception_state_channel::Channel{MyPerceptionType}
)
    while true
        # fetch(shutdown_channel) && break

        if isready(shutdown_channel)
            shutdown = take!(shutdown_channel)
            put!(shutdown_channel, shutdown)  # re-insert the flag so other tasks can see it
        
            if shutdown
                @info "Shutdown signal received"
                break
            end
        end

        fresh_gt_meas = GroundTruthMeasurement[]
        while isready(gt_channel)
            gt = take!(gt_channel)
            push!(fresh_gt_meas, gt)
        end

        if isempty(fresh_gt_meas)
            sleep(0.01)
            continue
        end

        # === Step 1: Get latest GT per vehicle ===
        latest_gt = Dict{Int, GroundTruthMeasurement}()

        for gt in fresh_gt_meas
            vid = gt.vehicle_id
            if !haskey(latest_gt, vid) || gt.time > latest_gt[vid].time
                latest_gt[vid] = gt
            end
        end

        # === Step 2: Ego Localization ===
        ego_vehicle_id = fresh_gt_meas[1].vehicle_id
        new_localization_state_from_gt = nothing

        if haskey(latest_gt, ego_vehicle_id)
            ego_gt = latest_gt[ego_vehicle_id]
            pos_x, pos_y, _ = ego_gt.position
            yaw = VehicleSim.extract_yaw_from_quaternion(ego_gt.orientation)
            new_localization_state_from_gt = LocalizationType(pos_x, pos_y, yaw)
        end

        # === Step 3: Perception of Other Vehicles ===
        estimated_states = EstimatedVehicleState[]

        for (veh_id, gt) in latest_gt
            if veh_id == ego_vehicle_id
                continue
            end

            pos_x, pos_y, pos_z = gt.position
            yaw = VehicleSim.extract_yaw_from_quaternion(gt.orientation)
            v = norm(gt.velocity)
            l, w, h = gt.size

            est = EstimatedVehicleState(
                veh_id,
                gt.time,
                pos_x,
                pos_y,
                pos_z,
                yaw,
                v,
                h, l, w  # watch order!
            )
            push!(estimated_states, est)
        end

        new_perception_state_from_gt = MyPerceptionType(
            maximum([gt.time for gt in values(latest_gt)]),
            estimated_states
        )

        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, new_localization_state_from_gt)
    
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, new_perception_state_from_gt)
    end
    @info "SHUTTING DOWN GT process"
end


# mutable struct RoadSegment
#     id::Int
#     lane_boundaries::Vector{LaneBoundary}
#     lane_types::Vector{LaneTypes}
#     speed_limit::Float64
#     children::Vector{Int}
# end

function estimate_velocity(pos, pos_prev, dt)
    dx = pos[1] - pos_prev[1]
    dy = pos[2] - pos_prev[2]
    return hypot(dx, dy) / dt
end

function estimate_steering_angle(yaw, yaw_prev, v, dt, L)
    dyaw = normalize_angle(yaw - yaw_prev)
    yaw_rate = dyaw / dt
    return atan(L * yaw_rate / (v + 1e-5))  # avoid div by zero
end

function decision_making(localization_state_channel, 
        perception_state_channel, 
        target_segment_channel,
        shutdown_channel,
        map_segments, 
        socket)

    path = nothing
    prev_id = nothing
    waypoints = nothing
    target_id = 0

    current_v = 0.0
    current_steering = 0.0
    prev_pos = nothing
    prev_yaw = nothing
    target_waypoint = nothing
    target_waypoint_ind = nothing
    next_segment = nothing
    prev_time = time()

    road_ids = nothing

    wait_time = nothing
    has_stopped_at_sign = false


    next_is_stop = false
    next_is_intersection = false
    is_in_or_approaching_intersection = false


        # ---------------------------------------------
    #  Fast‑turn parameters (used only when needed)
    # ---------------------------------------------
    MAX_STEER_INTER   = 1.20      #   69° hard limit
    MAX_RATE_INTER    = 2.50      # rad / s  (how fast the wheels may move)
    K_INTER           = 1.5      # Stanley gain

    MAX_STEER_NORMAL  = 0.70      #   40°
    MAX_RATE_NORMAL   = 1.00      # rad / s
    K_NORMAL          = 0.40
    wait_time = nothing

    # target_v = 1.0

    while true
        sleep(0.1)

        if isready(shutdown_channel)
            shutdown = take!(shutdown_channel)
            put!(shutdown_channel, shutdown)
            if shutdown
                @info "Shutdown signal received"
                break
            end
        end

        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        pos = (latest_localization_state.lat, latest_localization_state.long)
        target_v = 1.0
        # wait for stop sigsn
        if wait_time !== nothing
            if (wait_time - time()) > 0
                @info "WAITING"
                continue
            else
                wait_time = nothing
            end
        end

        if prev_pos === nothing
            prev_pos = pos
            continue  # wait one step to get a valid dt
        end

        dt = time() - prev_time
        prev_time = time()

        current_v = estimate_velocity(pos, prev_pos, dt)
        prev_pos = pos

        # estimate steering angle if not available
        yaw = latest_localization_state.yaw

        if prev_yaw !== nothing
            current_steering = estimate_steering_angle(yaw, prev_yaw, current_v, dt, 3)
        end

        prev_yaw = yaw

        while target_id == 0
            target_id = fetch(target_segment_channel)
            @info "Waiting for valid target_id, got: $target_id"
            sleep(0.1)
        end

        ego_id = find_road_id(map_segments, latest_localization_state)
        path = routing(map_segments, ego_id, target_id)
        waypoints, road_ids = DriveGoodAVStack.get_way_points_from_path(path, map_segments)
        prev_id = target_id

    


        cur_road_segment = map_segments[ego_id]
        speed_limit = cur_road_segment.speed_limit




 
        # --- Control ---
        k = K_NORMAL
        max_steering_rate = MAX_RATE_NORMAL # rad/s (smoother)
        max_accel = 1.0          # m/s²

        next_is_stop = false
        next_is_intersection = false

        _ ,target_waypoint_ind = stanley_control(latest_localization_state, current_v, waypoints; k=k)

        target_waypoint = waypoints[target_waypoint_ind]
        target_waypoint_road_id = road_ids[target_waypoint_ind]

        next_segment = map_segments[target_waypoint_road_id]
        next_segment_type_list = next_segment.lane_types

        for seg_typ in next_segment_type_list
            if seg_typ == VehicleSim.stop_sign
                next_is_stop = true
            elseif seg_typ == VehicleSim.intersection
                is_in_or_approaching_intersection = true
            else
                is_in_or_approaching_intersection = false
            end
        end

        for offset in 1:4
            lookahead_idx = target_waypoint_ind + offset
            if lookahead_idx <= length(road_ids)
                segment_id = road_ids[lookahead_idx]
                segment = map_segments[segment_id]
                if VehicleSim.intersection in segment.lane_types
                    is_in_or_approaching_intersection = true
                    break
                end
            end
        end
            
        if is_in_or_approaching_intersection
            k                 = K_INTER           # sharper correction
            max_steering_rate = MAX_RATE_INTER    # wheels can move faster
            max_steering_angle= MAX_STEER_INTER   # wheels can deflect further
            @info "USING INT CONSTANTS"
        else
            k                 = K_NORMAL
            max_steering_rate = MAX_RATE_NORMAL
            max_steering_angle= MAX_STEER_NORMAL
            @info "USING NORMAL"
        end

        

        cur_is_stop = VehicleSim.stop_sign in cur_road_segment.lane_types

        left_b  = cur_road_segment.lane_boundaries[1].pt_b
        right_b = cur_road_segment.lane_boundaries[2].pt_b
        segment_end = 0.5 .* (left_b + right_b)

        dist_to_end = hypot(segment_end[1] - pos[1],
                            segment_end[2] - pos[2])

        target_steering_angle, target_waypoint_ind = stanley_control(latest_localization_state, current_v, waypoints; k=k)
       
        steering_angle = smooth_steering(current_steering, target_steering_angle, max_steering_rate, dt)


        steering_angle  = clamp(steering_angle,
        -max_steering_angle,
        max_steering_angle)

      

       
        if cur_is_stop && dist_to_end < 3.0 && !has_stopped_at_sign
            target_v = 0.0
            wait_time = time() + 3
            has_stopped_at_sign = true
        elseif is_in_or_approaching_intersection && abs(steering_angle) > 0.2
            target_v = max(target_v, 1.2)  
        elseif is_in_or_approaching_intersection
            target_v = 1.0
        else
            target_v = 3.0
        end

        if !cur_is_stop || dist_to_end >= 3.0
            has_stopped_at_sign = false
        end


        @info "Next waypoint: $target_waypoint"
        @info "Next waypoint is intersection: $is_in_or_approaching_intersection"
        @info "Next waypoint is stop_sign: $next_is_stop "


        
        @info "current_v $current_v, target_v $target_v target_steering_angle $target_steering_angle"

        if target_v < current_v
            max_accel = 2.0
        else
            max_accel = 1.0
        end
        speed = smooth_velocity(current_v, target_v, max_accel, dt)
       
        cmd = (steering_angle, speed, true)
        serialize(socket, cmd)
        @info "Cmd sent: speed = $(round(speed, digits=2)), steer = $(round(steering_angle, digits=3))"
    end

    @info "SHUTTING DOWN DM process"
end

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end

function my_client(host::IPAddr=IPv4(0), port=4444; use_gt=false)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{LocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)
    target_segment_channel = Channel{Int}(1)
    shutdown_channel = Channel{Bool}(1)
    put!(shutdown_channel, false)


    put!(localization_state_channel, LocalizationType(0.0, 0.0, 0.0))
    put!(perception_state_channel, MyPerceptionType(0.0, []))

    # errormonitor(@async shutdown_listener(shutdown_channel))

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    put!(target_segment_channel, target_map_segment)

    errormonitor(@async while true
        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
        sleep(0.001)
        local measurement_msg
        received = false
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
                received = true
            else
                break
            end
        end
        !received && continue
        target_map_segment = measurement_msg.target_segment
        old_target_segment = fetch(target_segment_channel)
        if target_map_segment ≠ old_target_segment
            take!(target_segment_channel)
            put!(target_segment_channel, target_map_segment)
        end
        ego_vehicle_id = measurement_msg.vehicle_id
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end)

    if use_gt
        errormonitor(@async process_gt(gt_channel,
                      shutdown_channel,
                      localization_state_channel,
                      perception_state_channel))
    else
        errormonitor(@async localize(gps_channel, 
                    imu_channel, 
                    localization_state_channel, 
                    shutdown_channel))

        errormonitor(@async perception(cam_channel, 
                      localization_state_channel, 
                      perception_state_channel, 
                      shutdown_channel))
    end

    errormonitor(@async decision_making(localization_state_channel, 
                           perception_state_channel, 
                           target_segment_channel, 
                           shutdown_channel,
                           map_segments, 
                           socket))
    shutdown_listener(shutdown_channel)

end

function shutdown_listener(shutdown_channel)
    info_string = 
        "***************
      CLIENT COMMANDS
      ***************
            -Make sure focus is on this terminal window. Then:
            -Press 'q' to shutdown threads. 
    "
    @info info_string
    while true
        sleep(0.1)
        key = get_c()

        if key == 'q'
            @info "Starting shutdown ..."
            # terminate threads
            take!(shutdown_channel)
            put!(shutdown_channel, true)
            break
        end
    end
end