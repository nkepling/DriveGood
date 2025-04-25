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
    gps_noise = Diagonal([1, 1, 0.05])

    last_imu_time = first_imu.time
    last_gps_time = first_gps.time

    # send the very first estimate
    put!(localization_state_channel,
         LocalizationType(x_est[1], x_est[2], VehicleSim.extract_yaw_from_quaternion(x_est[4:7])))

    I13 = Matrix{Float64}(I, 13, 13)

    while true
        # —— imu prediction ——
        if isready(imu_channel)
            imu = take!(imu_channel)
            dt_imu = imu.time - last_imu_time
            last_imu_time = imu.time

            # load the newest IMU into your state
            x_est[8:10]  .= imu.linear_vel
            x_est[11:13] .= imu.angular_vel

            # 1) propagate mean and covariance
            x_est = VehicleSim.f(x_est, dt_imu)
            F     = VehicleSim.Jac_x_f(x_est, dt_imu)
            Cov_est = F*Cov_est*F' + state_noise
        end

        # —— GPS‐driven correction (non‐blocking) ——
        if isready(gps_channel)
            gps = take!(gps_channel)
            last_gps_time = gps.time

            # measurement vector
            z = [gps.lat, gps.long, gps.heading]

            # predicted measurement and Jacobian
            z_hat = VehicleSim.h_gps(x_est)
            H     = VehicleSim.Jac_h_gps(x_est)

            # Kalman update
            y = z .- z_hat
            # Normalize yaw residual to [-π, π]
            y[3] = mod(y[3] + π, 2π) - π 
            
            # Log residual and quaternion norm before update

            S = H*Cov_est*H' + gps_noise
            K = Cov_est*H'*inv(S)
            YAW_MAX = 0.15  # radians, e.g. ~30°
            if abs(y[3]) > YAW_MAX
                #@warn "Skipping yaw update (residual=$(y[3]))"
                 y[3] = sign(y[3]) * YAW_MAX
            end
            x_est += K * y


            # renormalize quaternion
            q_norm = norm(x_est[4:7])
            if q_norm < 1e-3
            #@warn "Quaternion norm too small — resetting to identity" q_norm=q_norm
                x_est[4:7] .= [1.0, 0.0, 0.0, 0.0]
            else
                x_est[4:7] ./= q_norm
            end
            Cov_est = (I13 - K*H)*Cov_est*(I13 - K*H)' + K*gps_noise*K'

        end

        # —— publish your fused pose at whatever rate you like —— 
        yaw = VehicleSim.extract_yaw_from_quaternion(x_est[4:7])
        if isready(localization_state_channel)
            take!(localization_state_channel)  # clear old
        end
        put!(localization_state_channel,
             LocalizationType(x_est[1]+3, x_est[2]-1, yaw))

        # check for shutdown
        if isready(shutdown_channel) && take!(shutdown_channel)
            @info "shutting down"
            break
        end

        # small sleep so we don’t spin <1ms
        sleep(1e-3)
    end
end
