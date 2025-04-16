using JLD2
using LinearAlgebra
using VehicleSim

include("../src/localization.jl")

function run_mse(file_path::String)
    # load data
    data = jldopen(file_path, "r") do file
        Dict(key => read(file, key) for key in keys(file))
    end
    for (k, v) in data
        println("key: $k")
        println("    type: ", typeof(v))
        println("    value: ", v)
        println()
    end

    # create channels
    gps_channel               = Channel{GPSMeasurement}(32)
    imu_channel               = Channel{IMUMeasurement}(32)
    localization_state_channel = Channel{LocalizationType}(1)
    shutdown_channel          = Channel{Bool}(1)

    msg = data["msg_buf"]
    println("Number of messages: ", length(msg))
    for i in 1:length(msg)
        for m in msg[i].measurements
        println(typeof(m), " => ", m)
        end
    end

    
    loc_task = @async my_localize(
        gps_channel,
        imu_channel,
        localization_state_channel,
        shutdown_channel
    )

    # arrs to store data 
    est_xs   = Float64[]
    est_ys   = Float64[]
    est_yaws = Float64[]
    gt_xs    = Float64[]
    gt_ys    = Float64[]
    gt_yaws  = Float64[]

    println("Loading measurements and collecting estimates...")

   
    num_to_process = min(20, length(msg))

    for i in 1:num_to_process
        for m in msg[i].measurements
            if m isa GPSMeasurement
                put!(gps_channel, m)

            elseif m isa IMUMeasurement
                put!(imu_channel, m)

            elseif m isa GroundTruthMeasurement
                # compare only if vehicle_id == 1
                if m.vehicle_id == 1
                    push!(gt_xs,  m.x)
                    push!(gt_ys,  m.y)
                    push!(gt_yaws, m.yaw)

                    # see if we have a new estimate
                    if isready(localization_state_channel)
                        est = take!(localization_state_channel)
                        push!(est_xs,  est.x)
                        push!(est_ys,  est.y)
                        push!(est_yaws, est.yaw)
                    end
                end
            end
        end
    end

    # shut down the localization loop
    put!(shutdown_channel, true)
    wait(loc_task)

    println("Number of ground-truth entries collected: ", length(gt_xs))
    println("Number of estimate entries collected:     ", length(est_xs))

    # compute MSE
    n = min(length(gt_xs), length(est_xs))
    if n == 0
        @warn "No overlapping ground truth and estimates collected!"
    else
        err_x   = 0.0
        err_y   = 0.0
        err_yaw = 0.0

        for i in 1:n
            dx   = gt_xs[i]   - est_xs[i]
            dy   = gt_ys[i]   - est_ys[i]
            dyaw = gt_yaws[i] - est_yaws[i]
            err_x   += dx^2
            err_y   += dy^2
            err_yaw += dyaw^2
        end

        mse_x   = err_x   / n
        mse_y   = err_y   / n
        mse_yaw = err_yaw / n

        println("Mean Squared Error:")
        println("  MSE in x   = $mse_x")
        println("  MSE in y   = $mse_y")
        println("  MSE in yaw = $mse_yaw")
    end
end
