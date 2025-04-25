
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
            new_localization_state_from_gt = LocalizationType(pos_x, pos_y, yaw, ego_gt.velocity)
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

function distance_to_end_of_segment(pos, seg)
    mid = [(seg.lane_boundaries[1].pt_b[i] + seg.lane_boundaries[2].pt_b[i]) / 2 for i in 1:2]
    return hypot(mid[1] - pos[1], mid[2] - pos[2])
end

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

function get_segment_end_point(seg::VehicleSim.RoadSegment)
    left, right = seg.lane_boundaries
    end_pt = [
        (left.pt_b[1] + right.pt_b[1]) / 2,
        (left.pt_b[2] + right.pt_b[2]) / 2
    ]
    return end_pt
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

    next_is_stop = false
    next_is_intersection = false
    is_in_or_approaching_intersection = false

    obstacles = false
    early_beak =false

        # ---------------------------------------------
    #  Fast‑turn parameters (used only when needed)
    # ---------------------------------------------
    MAX_STEER_INTER   = 1.20      #   69° hard limit
    MAX_RATE_INTER    = 2.50      # rad / s  (how fast the wheels may move)
    K_INTER           = 1.5      # Stanley gain

    MAX_STEER_NORMAL  = 1.00      #   40°
    MAX_RATE_NORMAL   = 4.0 #4.0     # rad / s
    K_NORMAL          = 0.4  # 1.0 gets us there renegade style
    wait_time = nothing

    has_stopped_at_sign = Dict()
    stop_start_time = Dict{Int, Float64}()

    has_stopped_at_loading = Dict{Int, Bool}()
    loading_start_time = Dict{Int, Float64}()
    # target_v = 1.0
    found_target  = false
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
        
        early_beak =false
        pos = (latest_localization_state.lat, latest_localization_state.long)
        current_v = norm(latest_localization_state.velo)
        target_v = 6.0
        # wait for stop sigsn
        # if wait_time !== nothing
        #     if (wait_time - time()) > 0
        #         @info "WAITING"
        #         continue
        #     else
        #         wait_time = nothing
        #         has_stopped_at_sign[ego_id] = true
        #     end
        # end

        if prev_pos === nothing
            prev_pos = pos
            continue  # wait one step to get a valid dt
        end

        dt = time() - prev_time
        prev_time = time()

        # current_v = estimate_velocity(pos, prev_pos, dt)
        # prev_pos = pos

        # estimate steering angle if not available
        yaw = latest_localization_state.yaw

        if prev_yaw !== nothing
            current_steering = estimate_steering_angle(yaw, prev_yaw, current_v, dt, 2.8)
        end

        prev_yaw = yaw

        proximity_thresh = 2.0  # meters
       
    
        while target_id == 0
            # target_id = fetch(target_segment_channel)
            target_id = 40
            found_target = false
            @info "Waiting for valid target_id, got: $target_id"
            sleep(0.1)
        end
        # grab current road_id
        ego_id = find_road_id(map_segments, latest_localization_state)
        
        if prev_id === nothing
            path = routing(map_segments, ego_id, target_id)
            waypoints, road_ids = DriveGoodAVStack.get_way_points_from_path(path, map_segments)
            prev_id = target_id
        end

        goal_dist = hypot(pos[1] - waypoints[end][1], pos[2] - waypoints[end][2])


        ### -- Get current road segmetn
        cur_road_segment = map_segments[ego_id]

        target_v = cur_road_segment.speed_limit - 3.5

        # get curvature 
        speed_limit = cur_road_segment.speed_limit
 
        # --- Control ---
        k = K_NORMAL
        max_steering_rate = MAX_RATE_NORMAL # rad/s (smoother)
        next_is_stop = false
        next_is_intersection = false

        

                # --- Existing Stanley controller output ---
        target_steering_angle, target_waypoint_ind = stanley_control(
            latest_localization_state, current_v, waypoints; k=k)

        target_waypoint = waypoints[target_waypoint_ind]

        target_waypoint_road_id = road_ids[target_waypoint_ind]
        

        # === ADD curvature feed-forward ===
        # κ = next_segment.curvature        # curvature field should be signed [1/m]
        #next_segment = map_segments[target_waypoint_road_id]
        cur_segment = map_segments[ego_id]
        left, right = cur_segment.lane_boundaries
        κ = abs(left.curvature) < abs(right.curvature) ? left.curvature : right.curvature



        #check for stop sign
        
        # next_is_stop = next_segment_id !== nothing && VehicleSim.stop_sign in map_segments[next_segment_id].lane_types

        



        L = 2.8                           # your wheelbase
        δ_ff = atan(L * κ)               # curvature feed-forward steering
        δ_fb = target_steering_angle     # stanley control term
        target_steering_angle = δ_fb + δ_ff

        # @info "δ_fb=$(round(δ_fb, digits=3)), δ_ff=$(round(δ_ff, digits=3)), δ_total=$(round(target_steering_angle, digits=3))"

        # @info "Curvature: $κ"
        steering_angle = smooth_steering(current_steering, target_steering_angle, max_steering_rate, dt)
  
        lookahead_κ = maximum([abs(map_segments[target_waypoint_road_id].lane_boundaries[1].curvature),abs(map_segments[target_waypoint_road_id].lane_boundaries[2].curvature)])
        
        if lookahead_κ > 1e-6
            target_v = min(target_v, 4.5)
        end

        # @info "lookahead_κ: $lookahead_κ"



        AY_MAX = 2.5  

        if abs(κ) > 1e-3 #&& target_steering_angle > 1e-3 # avoid div by near-zero

            MAX_STEER = 1.2 # rad
            L = 2.8

            # From bicycle model: R = L / tan(δ)
            max_κ_phys = tan(MAX_STEER) / L
            κ_safe = min(abs(κ), max_κ_phys)

            v_curve = sqrt(AY_MAX / κ_safe)
            target_v = min(target_v, v_curve)
        end

        if abs(κ) > 0.12 #&& target_steering_angle > 1e-3 # tighter than ~8m radius
            target_v = min(target_v, 5.0)
        end

   
    

        end_wp = get_segment_end_point(cur_segment)
        dist_to_end = hypot(pos[1] - end_wp[1], pos[2] - end_wp[2])
        stop_buffer_distance = 8.0  # meters before end of segment

        # @info "dist_to_end $dist_to_end"

        next_is_stop = VehicleSim.stop_sign in cur_segment.lane_types

        if !(ego_id in keys(has_stopped_at_sign))
            has_stopped_at_sign[ego_id] = false
        end

        if !(ego_id in keys(stop_start_time))
            stop_start_time[ego_id] = -1.0  
        end

        # # Start stop timer
     

        # If we're in the stop wait period

        if next_is_stop && !has_stopped_at_sign[ego_id] && current_v > 0.1
            target_v = 0.0
        end

        if next_is_stop && !has_stopped_at_sign[ego_id] && stop_start_time[ego_id] < 0 && current_v < 0.1
            stop_start_time[ego_id] = time()
        end


        if stop_start_time[ego_id] > 0 && (time() - stop_start_time[ego_id]) < 4.0 
            target_v = 0.0
            @info "Waiting at stop sign..."
        end

        # After waiting 4 seconds, we can resume
        if stop_start_time[ego_id] > 0 && (time() - stop_start_time[ego_id]) >= 4.0
            has_stopped_at_sign[ego_id] = true
            stop_start_time[ego_id] = -1.0
            @info "Done waiting at stop sign!"
        end

                # Initialize
        if !(ego_id in keys(has_stopped_at_loading))
            has_stopped_at_loading[ego_id] = false
        end
        if !(ego_id in keys(loading_start_time))
            loading_start_time[ego_id] = -1.0
        end

      


        if ego_id == target_id
            found_target = true
            @info "Found target!!!!!!!"
        end

        if found_target
            target_v = 0.0
        end
            
        if target_v < current_v
            max_accel = 3.0
        else
            max_accel = 3.0
        end

        if found_target 
            max_accel = 5.0
        end
        speed = smooth_velocity(current_v, target_v, max_accel, dt)
       
        cmd = (steering_angle, speed, true)
        serialize(socket, cmd)
        # @info "Next waypoint: $target_waypoint"
        # @info "Next waypoint is intersection: $is_in_or_approaching_intersection"
        @info "Next waypoint is stop_sign: $next_is_stop "
        @info "Curent road_id $ego_id"
        @info "PATH: $path"
        @info "current_v $current_v, target_v $target_v target_steering_angle $target_steering_angle"
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
    cam_meas_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{LocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)
    target_segment_channel = Channel{Int}(1)
    shutdown_channel = Channel{Bool}(1)
    put!(shutdown_channel, false)


    put!(localization_state_channel, LocalizationType(0.0, 0.0, 0.0,[0.0,0.0,0.0]))
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
                !isfull(cam_meas_channel) && put!(cam_meas_channel, meas)
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
        errormonitor(@async my_localize(gps_channel, 
                    imu_channel, 
                    localization_state_channel, 
                    shutdown_channel))

        errormonitor(@async perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel))
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