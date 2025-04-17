@testset "Stanley controller sanity check" begin
    map_segments = VehicleSim.city_map()

    starting_id = 35
    target_id = 79

    path = DriveGoodAVStack.routing(map_segments, starting_id, target_id)

    waypoints, road_ids = DriveGoodAVStack.get_way_points_from_path(path, map_segments)

    @test !isempty(waypoints)

    # Simulated vehicle state (starting just before the path)
    x, y = waypoints[1] .- 1.0  # a little behind the first waypoint
    yaw = 0.1                   # slight heading offset (in radians)
    v = 3.5                     # m/s

    loc = DriveGoodAVStack.LocalizationType(x,y,yaw)

    # Run Stanley controller
    δ = DriveGoodAVStack.stanley_control(loc, v, waypoints)

    @test isfinite(δ)
    @test -π/2 <= δ <= π/2  # sanity bounds on steering angle

    @info "Steering angle output: $δ radians"
end