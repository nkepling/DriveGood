
struct MyPerceptionType
    object_id::Int
    pos_x::Float64
    pos_y::Float64
    θ::Float64
    v::Float64
end

function perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    # set up stuff
    while true

        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)
        
        ego_x = latest_localization_state.postition[1]
        ego_y = latest_localization_state.postition[2]
        ego_z = latest_localization_state.position[3]

        #for now assume localizaiton is gt_type







        perception_state = MyPerceptionType(0,0.0)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)

        # moving to the end for testing purposes
        if fetch(shutdown_channel) == true
            @info "shutting down"
            flush(stdout)
            break
        end

    end
end