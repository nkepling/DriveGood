
# # Module usage
# using .GeometryUtils
# using .PurePursuit
# using .Planner
# using .DecisionMaking
# using .Routing
# using .GTProcessor  # If your process_gt function is inside a module

function my_client(host::IPAddr = IPv4(0), port = 4444; use_gt=true)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()

    # Initial setup message
    msg = deserialize(socket)
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

    # Measurement listener from socket
    @async begin
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

            # Dynamic routing
            route = routing(map_segments, ego_id, target_segment)
            polyline = construct_polyline_from_path(route, map_segments)
            put!(routing_channel, polyline)
        end
    end

    # ========== Choose Input Mode ==========
    if use_gt
        @async process_gt(gt_channel, shutdown_channel, localization_state_channel, perception_state_channel)
    else
        @async localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)

        @async perception(cam_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    end

    # ========== Planner Loop ==========
    @async begin
        state = :DRIVING
        stop_timer = 0.0
        stopped_at_time = 0.0
    
        while true
            fetch(shutdown_channel) && break
    
            loc = fetch(localization_state_channel)
            perception = fetch(perception_state_channel)
            path = fetch(routing_channel)
    
            @info "[Debug] Got localization", loc
            @info "[Debug] Got perception", perception
            @info "[Debug] Path size", length(path)
    
            obstacles = Planner.get_obstacle_positions(perception)
    
            steer, throttle, ok, state, stop_timer, stopped_at_time = plan_motion(loc, obstacles, path, state, stop_timer, stopped_at_time)
    
            cmd = VehicleCommand(steer, throttle, ok)
            serialize(socket, cmd)
    
            sleep(0.05)
        end
    end    
end

# 🚗 Run the system
# my_client(IPv4(0), 4444; use_gt=true)
