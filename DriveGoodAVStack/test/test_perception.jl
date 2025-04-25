using Test
using JLD2
using Infiltrator
using VehicleSim
using DriveGoodAVStack

@testset "test perception" begin
    # Load the JLD2 file as a dictionary
    # This data set has not empty CameraMeasurements.

    
    data = jldopen("/Users/nathankeplinger/Documents/Vanderbilt/Coursework/2025_Spring/software_for_autonomous_V/project/code/DriveGood/DriveGoodAVStack/test/msg_buffers/message_buff_2_car.jld2", "r") do file
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

    @info "Loading measuments from file"

    # 1 to 25 works
    #800:830

    for i in 1:4
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
    @info "Done Loading measurements from file"


    put!(shutdown_channel, true)

    # Call perception function with correct arguments

    @info "calling perception"
    DriveGoodAVStack.perception(
        cam_channel,
        localization_state_channel,
        perception_state_channel,
        shutdown_channel,
        gt_channel
    )

    # Test that the perception state is updated
    # @test isready(perception_state_channel)

    result = take!(perception_state_channel)
    @test !isempty(result.estimated_states) 

    @test result isa MyPerceptionType

    # @test length(result.estimated_states) == 2

    
    println(result)

    # how to remove 

    @info "Grabbing Gt"

    gt_measurements = []
    while isready(gt_channel)
        meas = take!(gt_channel)
        push!(gt_measurements, meas)
    end
    @info "Done Grabbing Measurements"
    

    last_gt = "None"
    last_size = "None"
    for g in gt_measurements
        if g.vehicle_id == 2
            last_gt_2 = g.position
            last_size = g.size
        end
    end

    # est_x = result.estimated_states[1].pos_x
    # est_y = result.estimated_states[1].pos_y
    # est_z = result.estimated_states[1].pos_z

    #compute MSE for all settinv 

    est = result.estimated_states

    # @info "Estimates: $est"

    

    @info "Gt positions; $gt_measurements"

    gt = gt_measurements[1].position

    println(gt[1])
    println(gt[2])
    println(est[1].pos_x)
    println(est[1].pos_y)

    gt_points = [gt[1],gt[2]]
    est_points = [est[1].pos_x,est[1].pos_y]

    function compute_mse(point1::Vector{Float64}, point2::Vector{Float64})
        @assert length(point1) == length(point2) "Points must have the same dimensions"
        mse = sum((point1 .- point2).^2) / length(point1)
        return mse
    end

    # @info "MSE $(compute_mse(gt_points,est_points))"
    
    
end
