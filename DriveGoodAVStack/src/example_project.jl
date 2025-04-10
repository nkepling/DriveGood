struct LocalizationType
    x::Int
    y::Int
    yaw::Float64
end

struct MyPerceptionType
    field1::Int
    field2::Float64
end

# How should we start localizaiton when program begins
# Random noise across the entire map?
# Figure out how to test estimation model

function localize(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
    # Read freshest gps data and and the use measurement and process model.
    #Need to create cov matrix for
    while true
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
        
        # process measurements
        # use the rigid_body_dynamics to predict state transition
        # calculate cov and estimate using math from slides
        # Then, use gps data gps to correct
        # Use more EKF math from slides
    

        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        # Finally, update the estimated state with our calculated one
        put!(localization_state_channel, localization_state)
    end 
end

function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)

        #=TODO:
        - Define our custom perception type
            - Timestamping
            - Assiging to track id
            - PoE values. 
        - Implement our EFK to handle noise in our BB state estimate
        - Implement helper function to do process camera measuments and extract state estimates
        - Implement "tracks" module that will handle the tracking of objecs overtime. 
            - This module will handle the logic for dealing with false positives, false negatives, and multiple detections.
            - Will compute probabilty of existence for each object in the track. 

        =#
        
        # process bounding boxes / run ekf / do what you think is good




        # TODO: Implementation notes..Simple first


        #= TODO: Inclass notes

        - Produce some kind of message that lets other components know where other vehicles are in 

        - vector of other vehcile states info, planal location, heading, velo, maybe size... 
        - maintain a list of vehicel states.. 
        - To start up populate some message from GT so that the decision making can use it. 'Get the piping done'
        - Next step start if detect bb say there is an obstacle say 10 meters in front of me. 

        EKF>>>>

        Dynamically spawn and and destroy EKF... So if you have two BBs in an image you need to keep track of two EKFs for each BB
        # No false positives ===> No cars there is not BBs 
        # All cars are the same height.. 
        # so all we need to know is [x,y ,θ, v] (uknown state: stuff we estimate with EKF)
        # the known state =  [z, l, w, h] , big rectangles that slide around (constants that we set and can dig around the source code and find)
        # Need an init prior for [x,y ,θ, v]... the height can be in full field of view. So if i know size of car, and pixels, you get get a guess of the depth.. 
        =#

        perception_state = MyPerceptionType(0,0.0)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end

function decision_making(localization_state_channel, 
        perception_state_channel, 
        map, 
        target_road_segment_id, 
        socket)
    # do some setup
    while true
        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        # Use MPC to plan trajectory based on latest state and map route
        # (steering_angle, target_vel) = mpc_plan(latest_localization_state, latest_perception_state, map, target_road_segment_id)

        # figure out what to do ... setup motion planning problem etc
        #drive slowly, steering straight
        steering_angle = 0.0
        target_vel = 2.0
        cmd = (steering_angle, target_vel, true)
        serialize(socket, cmd)
    end
end

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    # Buffer to log data steams for testing purposes.... 
    gps_buffer = Vector{GPSMeasurement}()
    imu_buffer = Vector{IMUMeasurement}()
    cam_buffer = Vector{CameraMeasurement}()
    gt_buffer = Vector{GroundTruthMeasurement}()


    #localization_state_channel = Channel{MyLocalizationType}(1)
    #perception_state_channel = Channel{MyPerceptionType}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    errormonitor(@async while true
        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
        sleep(0.001)
        local measurement_msg
        received = false
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
                received = true
            else
                break
            end
        end
        !received && continue
        target_map_segment = measurement_msg.target_segment
        ego_vehicle_id = measurement_msg.vehicle_id
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                push!(gps_buffer,meas)
                #!isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                push!(imu_buffer,meas)
                #!isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                push!(cam_buffer,meas)
                #!isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                push(gt_buffer,meas)
                #!isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end)
    jldsave("mybuffers.jld2";gps_buffer,imu_buffer,cam_buffer,gt_buffer)

    return

    @async localize(gps_channel, imu_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket)
end
