using Test
using JLD2

@testset "test perception" begin
    # Load the JLD2 file as a dictionary
    # This data set has not empty CameraMeasurements.
    data = jldopen("/Users/nathankeplinger/Documents/Vanderbilt/Coursework/2025_Spring/software_for_autonomous_V/project/code/DriveGood/DriveGoodAVStack/tests/msg_buffers/stopped_vehicle_preception_message_buff.jld2", "r") do file
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

    msg = data["msg"]

    num_msg = length(msg)

    # load the fist 32 measurements # fill the localzation with ground truth for now. 
    for i in 1:32
        put!(cam_channel,msg[i].measurements[3])
        put!(cam_channel,msg[i].measurements[4])
        # this is the other vehicle
        put!(gt_channel, msg[i].measurements[6])
        if i == 32
            put!(localization_state_channel, msg[i].measurements[5])
        end
    end 

    put!(shutdown_channel, true)

    # Call perception function with correct arguments
    DriveGoodAVStack.perception.perception(
        cam_channel,
        localization_state_channel,
        perception_state_channel,
        shutdown_channel
    )

    # Test that the perception state is updated
    @test isready(perception_state_channel)
    result = take!(perception_state_channel)
    @test result isa MyPerceptionType
end
