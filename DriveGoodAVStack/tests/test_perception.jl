using Test
using JLD2

@testset "test perception" begin
    # Load the JLD2 file as a dictionary
    # This data set has not empty CameraMeasurements.
    data = jldopen("--", "r") do file
        Dict(key => read(file, key) for key in keys(file))
    end

    # Create channels
    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    #localization_state_channel = Channel{MyLocalizationType}(1)
    localization_state_channel = Channel{GroundTruthMeasurement}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)
    shutdown_channel = Channel{Bool}(1)
    info_channel = Channel{Any}(32)

    # Seed channels with test data

    msg = data["msg_buf"]

    num_msg = length(msg)

    println("Loading measuments from file")

    for i in 1:15
        for m in msg[i].measurements
            if m isa CameraMeasurement
    
                try
                    put!(cam_channel, m)
                catch e
                    println("Warning: cam_channel is full, skipping this measurement.")
                end
            elseif m isa GroundTruthMeasurement
                if m.vehicle_id == 1
                    try
                        if isready(localization_state_channel)
                            take!(localization_state_channel)
                        end
                        put!(localization_state_channel, m)
                    catch e
                        println("Warning: localization_state_channel is full, skipping this measurement.")
                    end
                else
                    try
                        put!(gt_channel, m)
                    catch e
                        println("Warning: gt_channel is full, skipping this measurement.")
                    end
                end
            else
                # println("Unknown measurement type: $m")
                continue
            end
        end
    end
    println("Done Loading measurements from file")

    
    put!(shutdown_channel, true)

    # Call perception function with correct arguments
    println("calling perception")
    DriveGoodAVStack.perception(
        cam_channel,
        localization_state_channel,
        perception_state_channel,
        shutdown_channel
    )

    # Test that the perception state is updated
    @test isready(perception_state_channel)
    result = take!(perception_state_channel)
    @test result isa MyPerceptionType
    println(result)
end
