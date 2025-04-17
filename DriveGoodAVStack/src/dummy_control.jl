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




function decision_making(localization_state_channel, 
        perception_state_channel, 
        target_segment_channel,
        shutdown_channel,
        map_segments, 
        socket)
    # do some setup

    path = nothing
    prev_id = nothing
    target_id = 0

    current_v = 0.0
    while true
        sleep(0.1)
   

        if isready(shutdown_channel)
            shutdown = take!(shutdown_channel)
            put!(shutdown_channel, shutdown)  # re-insert the flag so other tasks can see it

            if shutdown
                @info "Shutdown signal received"
                break
            end
        end

        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        while target_id == 0
            target_id = fetch(target_segment_channel)
            @info "Waiting for valid target_id, got: $target_id"
            sleep(0.1)  # prevent busy spinning
        end

        if prev_id === nothing
            ego_id = find_road_id(map_segments,latest_localization_state)
            @info "Ego road segment ID: $ego_id"
            path,road_ids = get_path(map_segments,ego_id,target_id)
            prev_id = target_id
        elseif target_id != prev_id
            ego_id = find_road_id(map_segments,latest_localization_state)
            @info "Ego road segment ID: $ego_id"
            path,road_ids = get_path(map_segments,ego_id,target_id)
            prev_id = target_id
        end

        ego_id = find_road_id(map_segments,latest_localization_state)
        cur_road_segment= map_segments[ego_id]
        seg_type = cur_road_segment.lane_types
        speed_limit = cur_road_segment.speed_limit




        # need a velo estimate
        @info "call pure_pursuit"
        #the actual method does not work but the piping is there...
        # steering_angle,target_vel = pure_pursuit(latest_localization_state,current_v,path;ls=0.5,L=2)
        # target_vel = min(target_vel,speed_limit/2)
        # Get list of stop sign ids/coodinate

        # DO PURE PUSUIT TAKE IN CURRENT POSTION AND RETURN (steering, target_velo,true)
        # streeing_angle,target_vel = pure_pursuit()
        #
        # check if reached target_id. if so pause. 

        # if obstacles with 3 meters stop and wait for some times. After time elapese check again

        # figure out what to do ... setup motion planning problem etc

        cmd = (steering_angle, target_vel, true)
        current_v = target_vel
        # @info "Sending command: ", cmd
        serialize(socket, cmd)
        @info "sending cmd: target_vel: $target_vel, steering_angle: $steering_angle"

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