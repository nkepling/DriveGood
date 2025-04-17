using LinearAlgebra
using VehicleSim
function localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)

    # State vector x: [p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, ω_x, ω_y, ω_z]
    x_est = zeros(13)
    # quaternion is identity 
    x_est[4] = 1.0

    # initial cov
    Cov_est = Matrix{Float64}(I, 13, 13) * 1e-2

    # noise cov matrix 
    state_noise = Matrix{Float64}(I, 13, 13) * 1e-3

    # GPS meas noise cov. GPS[lat, long, heading].
    gps_noise = Diagonal([1.0, 1.0, 0.1])

    # system time to compute dt
    last_time = time()

    while true
        # collect fresh measurements 
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end

        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end

        # for simplicity, derive dt from the system clock
        current_time = time()
        dt = current_time - last_time
        last_time = current_time

        # ========== EKF prediction step ==========

        # use the process model to predict new state
        x_pred = VehicleSim.f(x_est, dt)

        # Evaluate the jac of f at the current state
        F = VehicleSim.Jac_x_f(x_est, dt)

        # covariance calc
        Cov_pred = F * Cov_est * transpose(F) + state_noise

        # ===========================================

        # ========== EKF Update (GPS) ==========

        # choose the latest measurement
        if !isempty(fresh_gps_meas)
            gps_meas = fresh_gps_meas[end]

            # maybe unneeded no idea, depends if gps meas is sending type or vector
            z_gps = [gps_meas.lat, gps_meas.long, gps_meas.heading]
            
            # predicted measurement using our process model output.
            z_hat = VehicleSim.h_gps(x_pred)
            # measurement Jacobian evaluated at the predicted state.
            H = VehicleSim.Jac_h_gps(x_pred)
            
            # Compute residual between actual and predicted
            y = z_gps .- z_hat
            # Compute residual cov
            S = H * Cov_pred * transpose(H) + gps_noise
            # Kalman gain.
            K = Cov_pred * transpose(H) * inv(S)
            # Update state estimate.
            x_update = x_pred .+ K * y
            # Normalize the quaternion to maintain a unit quaternion.
            q_norm = norm(x_update[4:7])
            x_update[4:7] .= x_update[4:7] ./ q_norm

            # Update the cov matrix
            I13 = Matrix{Float64}(I, 13, 13)
            P_update = (I13 - K * H) * P_pred * transpose(I13 - K * H) + K * R_gps * transpose(K)
        else
            # if no new GPS measurement is available
            x_update = x_pred
            P_update = P_pred
        end

        # updates for next prediction
        x_est .= x_update
        Cov_est .= P_update

        # push the localization estimate
        
        yaw = VehicleSim.extract_yaw_from_quaternion(x_est[4:7])
        estimated_state = LocalizationType(round(Int, x_est[1]),round(Int, x_est[2]), yaw)

        # clear old state on the channel
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        # send out new estimate
        put!(localization_state_channel, estimated_state)

        # sleep briefly
        sleep(0.01)

        if fetch(shutdown_channel) == true
            @info "shutting down"
            flush(stdout)
            break
        end
    end 
end