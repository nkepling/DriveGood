
function run_planner(loc_channel, perception_channel, path_channel, socket, shutdown_channel)
    @info "Motion planner running..."

    while true
        fetch(shutdown_channel) && break

        #  Fetch state + perception
        loc = fetch(loc_channel)
        obs = fetch(perception_channel)

        #  Fetch the full rich path with stop/pullout flags
        rich_path = fetch(path_channel)
        # Format: [(SVector(x, y), stop_flag::Bool, pullout_flag::Bool), ...]

        #  Plan motion
        steer, throttle, active = Planner.plan_motion(loc, obs, rich_path)

        #  Send command to VehicleSim
        cmd = VehicleCommand(steer, throttle, active)
        serialize(socket, cmd)

        sleep(0.02)
    end
end
