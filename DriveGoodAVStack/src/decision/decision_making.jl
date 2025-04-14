module DecisionMaking

export run_planner

using ..Planner

function run_planner(loc_channel, perception_channel, path_channel, socket, shutdown_channel)
    @info "Motion planner running..."

    while true
        fetch(shutdown_channel) && break

        loc = fetch(loc_channel)
        obs = fetch(perception_channel)
        path = fetch(path_channel)

        steer, throttle, active = Planner.plan_motion(loc, obs, path)

        cmd = VehicleCommand(steer, throttle, active)
        serialize(socket, cmd)

        sleep(0.02)
    end
end

end
