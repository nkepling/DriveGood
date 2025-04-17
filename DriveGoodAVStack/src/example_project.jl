using Sockets
using Serialization
using VehicleSim
using LinearAlgebra

include("decision/decision_making.jl")

function process_gt(gt_channel, shutdown_channel, localization_state_channel, perception_state_channel)
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

        dummy_obstacles = []  # Or put something fake if you want to test avoidance
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, dummy_obstacles)

        @info "[GT] Updated localization and perception"
    end
end

function my_client(host::IPAddr = IPv4(0), port = 4444)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()

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

            put!(routing_channel, [
    ((-90.0, -78.0), false, false),
    ((-85.0, -75.0), true, false),
    ((-80.0, -72.0), false, true)
])



        end
    end)

    errormonitor(@async process_gt(gt_channel, shutdown_channel, localization_state_channel, perception_state_channel))

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
