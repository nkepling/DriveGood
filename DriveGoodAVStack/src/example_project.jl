



# function process_gt(
#     gt_channel,
#     shutdown_channel,
#     localization_state_channel,
#     perception_state_channel)


#     while true
#         if isready(shutdown_channel)
#             fetch(shutdown_channel) && break
#         end


#         fresh_gt_meas = []
#         while isready(gt_channel)
#             meas = take!(gt_channel)
#             push!(fresh_gt_meas, meas)
#         end

#         # process the fresh gt_measurements to produce localization_state and
#         # perception_state
        
#         take!(localization_state_channel)
#         put!(localization_state_channel, new_localization_state_from_gt)
        
#         take!(perception_state_channel)
#         put!(perception_state_channel, new_perception_state_from_gt)
#     end
# end


# function process_gt(
#     gt_channel,
#     shutdown_channel,
#     localization_state_channel,
#     perception_state_channel)


#     while true
#         if isready(shutdown_channel)
#             fetch(shutdown_channel) && break
#         end


#         fresh_gt_meas = []
#         while isready(gt_channel)
#             meas = take!(gt_channel)
#             push!(fresh_gt_meas, meas)
#         end

#         # process the fresh gt_measurements to produce localization_state and
#         # perception_state
        
#         take!(localization_state_channel)
#         put!(localization_state_channel, new_localization_state_from_gt)
        
#         take!(perception_state_channel)
#         put!(perception_state_channel, new_perception_state_from_gt)
#     end
# end

# # TODO decide if using bicycle model or point model
# # How should we start localizaiton when program begins
# # Random noise across the entire map?
# # Figure out how to test estimation model


# # function perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel)
# #     # set up stuff
# #     while true

# #         if fetch(shutdown_channel) == true
# #             @info "shutting down"
# #             flush(stdout)
# #             break
# #         end
# #         fresh_cam_meas = []
# #         while isready(cam_meas_channel)
# #             meas = take!(cam_meas_channel)
# #             push!(fresh_cam_meas, meas)
# #         end

# #         latest_localization_state = fetch(localization_state_channel)

# #         perception_state = MyPerceptionType(0,0.0)
# #         if isready(perception_state_channel)
# #             take!(perception_state_channel)
# #         end
# #         put!(perception_state_channel, perception_state)
# #     end
# # end

# function decision_making(localization_state_channel, 
#         perception_state_channel, 
#         shutdown_channel,
#         map, 
#         target_road_segment_id, 
#         socket)
#     # do some setup
#     while true
#         fetch(shutdown_channel) && break # if shutudown state_channel is 1 then break 
#             @info "Shutting down localization thread."
#         latest_localization_state = fetch(localization_state_channel)
#         latest_perception_state = fetch(perception_state_channel)

#         # Use MPC to plan trajectory based on latest state and map route
#         # (steering_angle, target_vel) = mpc_plan(latest_localization_state, latest_perception_state, map, target_road_segment_id)

#         # figure out what to do ... setup motion planning problem etc
#         #drive slowly, steering straight
#         steering_angle = 0.0
#         target_vel = 2.0
#         cmd = (steering_angle, target_vel, true)
#         serialize(socket, cmd)
#     end
# end

# function isfull(ch::Channel)
#     length(ch.data) ≥ ch.sz_max
# end


# function shutdown_listener(shutdown_channel,info_channel)
#     println("Starting shutdown listener")
#     info_string = 
#         "***************
#       CLIENT COMMANDS
#       ***************
#             -Make sure focus is on this terminal window. Then:
#             -Press 'q' to shutdown threads. 
#     "
#     @info info_string
#     while true
#         sleep(0.1)
#         key = get_c()

#         if key == 'q'
#             # terminate threads
#             @info "Shut down signal received"
#             flush(stderr)
#             take!(shutdown_channel)
#             put!(shutdown_channel, true)
#             break
#         end
#     end
#     @info "Shutdown signal sent. Waiting for threads to exit."
# end


# function my_client(host::IPAddr=IPv4(0), port=4444,use_gt=false)

#     @info "Staring Client"
#     socket = Sockets.connect(host, port)
#     map_segments = VehicleSim.city_map()

#     # Initial setup message
#     msg = deserialize(socket)
#     while msg isa String
#         @info "Skipping string message: $msg"
#         msg = deserialize(socket)
#     end

#     ego_id = msg.vehicle_id
#     target_segment = msg.target_segment

#     @info "Connected to VehicleSim. Vehicle ID: $ego_id"

#     # Channels
#     gps_channel = Channel{GPSMeasurement}(32)
#     imu_channel = Channel{IMUMeasurement}(32)
#     cam_channel = Channel{CameraMeasurement}(32)
#     gt_channel = Channel{GroundTruthMeasurement}(32)


#     localization_state_channel = Channel{MyLocalizationType}(1)
#     perception_state_channel = Channel{MyPerceptionType}(1)
#     target_segment_channel = Channel{Int}(1)
#     shutdown_channel = Channel{Bool}(1)
#     info_channel = Channel{Any}(32)
#     put!(shutdown_channel, false)   

#     errormonitor(@async shutdown_listener(shutdown_channel,info_channel))



#     # Buffer to log data steams for testing purposes.... 
#     gps_buffer = Vector{GPSMeasurement}()
#     imu_buffer = Vector{IMUMeasurement}()
#     cam_buffer = Vector{CameraMeasurement}()
#     gt_buffer = Vector{GroundTruthMeasurement}()


#     #localization_state_channel = Channel{MyLocalizationType}(1)
#     #perception_state_channel = Channel{MyPerceptionType}(1)

#     target_map_segment = 0 # (not a valid segment, will be overwritten by message)
#     ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)


#     # set_up shutdown shutdown_listener

#     errormonitor(@async while true
#         # This while loop reads to the of the socket stream (makes sure you
#         # are looking at the latest messages)
#         sleep(0.001)
#         local measurement_msg
#         received = false
#         while true
#             @async eof(socket)
#             if bytesavailable(socket) > 0
#                 measurement_msg = deserialize(socket)
#                 received = true
#             else
#                 break
#             end
#         end

#         !received && continue
#         target_map_segment = measurement_msg.target_segment
#         old_target_segment = fetch(target_segment_channel)
#         if target_map_segment ≠ old_target_segment
#             take!(target_segment_channel)
#             put!(target_segment_channel, target_map_segment)
#         end
#         ego_vehicle_id = measurement_msg.vehicle_id
#         for meas in measurement_msg.measurements
#             if meas isa GPSMeasurement
#                 # push!(gps_buffer,meas)
#                 !isfull(gps_channel) && put!(gps_channel, meas)
#             elseif meas isa IMUMeasurement
#                 # push!(imu_buffer,meas)
#                 !isfull(imu_channel) && put!(imu_channel, meas)
#             elseif meas isa CameraMeasurement
#                 #push!(cam_buffer,meas)
#                 !isfull(cam_channel) && put!(cam_channel, meas)
#             elseif meas isa GroundTruthMeasurement
#                 #push(gt_buffer,meas)
#                 !isfull(gt_channel) && put!(gt_channel, meas)
#             end
#         end
#     end)
#     #jldsave("mybuffers.jld2";gps_buffer,imu_buffer,cam_buffer,gt_buffer)


#     if use_gt
#         @async process_gt(gt_channel,
#                       shutdown_channel,
#                       localization_state_channel,
#                       perception_state_channel)
#     else
#         @async localize(gps_channel, 
#                     imu_channel, 
#                     localization_state_channel, 
#                     shutdown_channel)

#         @async perception(cam_channel, 
#                       localization_state_channel, 
#                       perception_state_channel, 
#                       shutdown_channel)
#     end

#     # @async decision_making(localization_state_channel, perception_state_channel,shutdown_channel, map, target_road_segment_id, socket)
# end


