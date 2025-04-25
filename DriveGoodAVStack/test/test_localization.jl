using JLD2
using LinearAlgebra
using VehicleSim

include("../src/localization.jl")
function run_mse(file_path::String)
    # load data
    data = jldopen(file_path, "r") do file
        Dict(key => read(file, key) for key in keys(file))
    end

    gps_channel               = Channel{GPSMeasurement}(32)
    imu_channel               = Channel{IMUMeasurement}(32)
    localization_state_channel = Channel{LocalizationType}(1)
    shutdown_channel          = Channel{Bool}(1)

    msg = data["msg_buf"]

    loc_task = @async my_localize(
        gps_channel,
        imu_channel,
        localization_state_channel,
        shutdown_channel
    )

    est_xs   = Float64[]
    est_ys   = Float64[]
    est_yaws = Float64[]
    gt_xs    = Float64[]
    gt_ys    = Float64[]
    gt_yaws  = Float64[]

    println("Loading measurements and collecting estimates...")

    num_to_process =   length(msg)

    for i in 1:num_to_process
        # Push all measurements from msg[i]
        for m in msg[i].measurements
            if m isa GPSMeasurement
                put!(gps_channel, m)

            elseif m isa IMUMeasurement
                put!(imu_channel, m)

            elseif m isa GroundTruthMeasurement
                # Compare only if vehicle_id == 1
                if m.vehicle_id == 1
                    push!(gt_xs,  m.position[1])
                    push!(gt_ys,  m.position[2])
                    yaw = VehicleSim.extract_yaw_from_quaternion(m.orientation)
                    push!(gt_yaws, yaw)

                    # Here we can try to grab any new estimates that
                    # the localization task might have produced
                    while isready(localization_state_channel)
                        est = take!(localization_state_channel)
                        push!(est_xs,  est.lat)
                        push!(est_ys,  est.long)
                        push!(est_yaws, est.yaw)
                    end
                end
            end
        end

        # Give the localization task a chance to run its loop
        # so it can consume the above measurements and produce an estimate.
        # A small sleep or yield() will do.
        sleep(0.01)
        # or just yield()
        # yield()
    end

    # Let the localization task finish processing any final measurements
    sleep(0.5)

    # Now shut it down
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
        print("x_gt,y_gt,x_est,y_est,yaw_gt,yaw_est\n")

        for i in 1:n
            print(gt_xs[i])
            print(",")
            print(gt_ys[i])
            print(",")
            print(est_xs[i])
            #print("y est ")
            #
            print(",")
            print(est_ys[i])
            print(",")
            #print("yaw est ")
            print(gt_yaws[i])
            print(",")
            print(est_yaws[i])
            print("\n")
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
