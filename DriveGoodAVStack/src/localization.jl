using LinearAlgebra
using VehicleSim

struct LocalizationType
    lat::Float64
    long::Float64
    yaw::Float64
end

function my_localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)

    # State vector x: [p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, ω_x, ω_y, ω_z]
    while !isready(gps_channel) & !isready(imu_channel)
        sleep(0.01)
    end
    first_gps = take!(gps_channel)
    first_imu = take!(imu_channel)

    x_est = zeros(13)
    # quaternion is identity 

    x_est[1] = first_gps.lat
    x_est[2] = first_gps.long
    yaw = first_gps.heading
    x_est[4] = cos(yaw / 2)
    x_est[5] = 0.0
    x_est[6] = 0.0
    x_est[7] = sin(yaw / 2)
    x_est[8] = first_imu.linear_vel[1]
    x_est[9] = first_imu.linear_vel[2]
    x_est[10] = first_imu.linear_vel[3]
    x_est[11] = first_imu.angular_vel[1]
    x_est[12] = first_imu.angular_vel[2]
    x_est[13] = first_imu.angular_vel[3]
    

    # initial cov
    Cov_est = Matrix{Float64}(I, 13, 13) * 1e-2

    # noise cov matrix 
    state_noise = Matrix{Float64}(I, 13, 13) * 1e-3

    # GPS meas noise cov. GPS[lat, long, heading].
    gps_noise = Diagonal([1, 1, 0.1])

    # system time to compute dt
    last_time = first_gps.time
    last_gps = first_gps
    last_imu =first_imu

    # ================= Main Loop =========================
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
    
        if !isempty(fresh_gps_meas)
            last_gps = fresh_gps_meas[end]
        end

        if !isempty(fresh_imu_meas)
            last_imu = fresh_imu_meas[end]
        end

        current_time = last_gps.time
        dt = current_time - last_time
        last_time = current_time

        x_pred = VehicleSim.f(x_est, dt)

        # Evaluate the jac of f at the current state
        F = VehicleSim.Jac_x_f(x_est, dt)

        # covariance calc
        Cov_pred = F * Cov_est * transpose(F) + state_noise

        z_gps = [last_gps.lat, last_gps.long, last_gps.heading]
        
        # predicted measurement using our process model output.
        z_hat = VehicleSim.h_gps(x_pred)
        # measurement Jacobian evaluated at the predicted state.
        H = VehicleSim.Jac_h_gps(x_pred)
        # Compute residual between actual and predicted
        y = z_gps .- z_hat
        #y = z_gps .- z_hat
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
        P_update = (I13 - K*H) * Cov_pred * transpose(I13 - K*H) + K * gps_noise * transpose(K)
        
        # # --- after  GPS x_update and P_update ---
        # # build the IMU “measurement”
        z_imu = vcat( last_imu.linear_vel,
        last_imu.angular_vel )        # 6×1

        # measurement Jacobian picks out states 8–13
        H_imu = zeros(6,13)
        H_imu[:, 8:13] .= I(6)

        # covariance on your IMU readings (tune these)
        R_imu = Diagonal(vcat(fill(1,3), fill(0.1,3)))

        # residual
        y_imu = z_imu .- x_update[8:13]

        # Kalman‐gain
        S_imu = H_imu * P_update * H_imu' + R_imu
        K_imu = P_update * H_imu' * inv(S_imu)

        # state & cov updates
        x_update = x_update .+ K_imu * y_imu
        P_update = (I - K_imu*H_imu) * P_update * (I - K_imu*H_imu)' + K_imu*R_imu*K_imu'

        # normalize quaternion again
        x_update[4:7] ./= norm(x_update[4:7])

        # write back
        x_est   .= x_update
        Cov_est .= P_update

        yaw = VehicleSim.extract_yaw_from_quaternion(x_est[4:7])
        estimated_state = LocalizationType(x_est[1],x_est[2], yaw)

        # clear old state on the channel
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        # send out new estimate
        put!(localization_state_channel, estimated_state)
        

        # sleep briefly
        sleep(0.01)

        if isready(shutdown_channel) && take!(shutdown_channel) == true
            @info "shutting down"
        break
        end
    end 
end
