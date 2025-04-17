function process_gt(
    gt_channel::Channel{GroundTruthMeasurement},
    shutdown_channel::Channel{Bool},
    localization_state_channel::Channel{LocalizationType},
    perception_state_channel::Channel{MyPerceptionType}
)
    while true
        fetch(shutdown_channel) && break

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

        # === Step 4: Send results ===
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, new_localization_state_from_gt)

        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
     
    end
    @info "SHUTTING DOWN GT process"
end


function decision_making(localization_state_channel, 
        perception_state_channel, 
        target_segment_channel,
        shutdown_channel,
        map, 
        socket)
    # do some setup


    while true

        fetch(shutdown_channel) && break
        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)
        # figure out what to do ... setup motion planning problem etc
        steering_angle = 0.0
        target_vel = 10.0
        cmd = (steering_angle, target_vel, true)
        @info "Sending command: ", cmd
        serialize(socket, cmd)
    end
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
                           map, 
                           socket))

    return shutdown_channel
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