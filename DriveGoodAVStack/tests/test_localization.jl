using JLD2
using LinearAlgebra
using VehicleSim  # This imports your measurement types and the localize function.
using Sockets

# -- load data -------------------------------------------------------
data = jldopen("DriveGoodAVStack/jld2/message_buff.jld2", "r") do file
    Dict(key => read(file, key) for key in keys(file))
end

msgs = data["msg"]
num_msgs = length(msgs)
if num_msgs == 0
    println("No messages found in the JLD2 file!")
    exit(1)
end

# -- Channels -------------------------------------------------------
gps_channel = Channel{GPSMeasurement}(32)
imu_channel = Channel{IMUMeasurement}(32)
localization_state_channel = Channel{VehicleSim.LocalizationType}(1)
shutdown_channel = Channel{Bool}(1)


gt_positions = Vector{Vector{Float64}}()

# extract ground-truth measurement and sensor channels
for i in 1:min(32, num_msgs)
    gt_meas = msgs[i].measurements[6] :: GroundTruthMeasurement
    # Convert the ground-truth position (assumed to be an SVector) to a built-in array.
    pos = collect(gt_meas.position)
    push!(gt_positions, pos)
    
    # Create a simulated GPS measurement using the ground-truth x and y values.
    gps_meas = GPSMeasurement(gt_meas.time, pos[1], pos[2], 0.0)
    put!(gps_channel, gps_meas)
    
    # Create a dummy IMU measurement (all zeros) using built-in arrays.
    dummy_imu = IMUMeasurement(gt_meas.time, zeros(3), zeros(3))
    put!(imu_channel, dummy_imu)
end

# -- Run Localization -----------------------------------------------------
# Launch the localization routine asynchronously.
loc_task = @async localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)

# Allow the localization routine some time to process the sensor data.
sleep(0.5)

# Retrieve the latest localization estimate.
est = take!(localization_state_channel)
# Assume the estimate is of type LocalizationType with integer fields x and y.
estimated_xy = [est.x, est.y]

# Compare the estimated (x,y) with the ground-truth (x,y) from the last seeded message.
gt_last = gt_positions[end]

dx = estimated_xy[1] - gt_last[1]
dy = estimated_xy[2] - gt_last[2]
mse = dx^2 + dy^2

println("Mean Squared Error (MSE): ", mse)

# -- Shutdown -------------------------------------------------------------
put!(shutdown_channel, true)
wait(loc_task)
