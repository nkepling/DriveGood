function process_gt(gt_channel, shutdown_channel, localization_state_channel, perception_state_channel)
    while true
        fetch(shutdown_channel) && break

        fresh_gt_meas = []
        while isready(gt_channel)
            meas = take!(gt_channel)
            push!(fresh_gt_meas, meas)
        end

        # === Generate fake localization and perception ===
        if !isempty(fresh_gt_meas)
            latest = fresh_gt_meas[end]

            # Construct localization data
            loc = (
                position = (latest.pos_x, latest.pos_y, latest.pos_z),
                heading = latest.θ,
                velocity = (latest.v * cos(latest.θ), latest.v * sin(latest.θ), 0.0),
            )

            # Construct dummy obstacles (this would usually be extracted from bounding boxes etc.)
            dummy_obstacles = [(latest.pos_x + 10.0, latest.pos_y + 2.0)]

            @info "[GT] Sending loc", loc
            @info "[GT] Sending obs", dummy_obstacles

            if isready(localization_state_channel)
                take!(localization_state_channel)
            end
            put!(localization_state_channel, loc)

            if isready(perception_state_channel)
                take!(perception_state_channel)
            end
            put!(perception_state_channel, dummy_obstacles)
        end

        sleep(0.05)
    end
end
