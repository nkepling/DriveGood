
using Test
using JLD2
using Infiltrator
using VehicleSim
using DriveGoodAVStack

function run_perception!(cam, loc, perc, shut, gt, results)
    put!(shut, true)                      # tell perception we’re done
    DriveGoodAVStack.perception(cam, loc, perc, shut, gt)
    result = take!(perc)
    push!(results, result)
end

@testset "full_suite_perception_test" begin

    #static vehicle test 
    data = jldopen("/Users/nathankeplinger/Documents/Vanderbilt/Coursework/2025_Spring/software_for_autonomous_V/project/code/DriveGood/DriveGoodAVStack/test/msg_buffers/stopped_vehicle_preception_message_buff.jld2", "r") do file
        Dict(key => read(file, key) for key in keys(file))
    end

    msgs = data["msg_buf"]

 

    i = 1

    all_results = MyPerceptionType[]
    
    while i <= length(msgs)
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



    end



    # basic data type test


    #Two static vehicle test

    # Moving ego vehicle test


    # Object drive by test


end

