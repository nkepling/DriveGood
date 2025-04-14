using Test
using StaticArrays
include("../utils/geometry.jl")
include("../planner/pure_pursuit.jl")
include("../planner/planner.jl")

using .GeometryUtils
using .PurePursuit
using .Planner

@testset "Planner and Pure Pursuit" begin
    ## === Test 1: Straight Path === ##
    path = [SVector(x, 0.0) for x in 0.0:1.0:10.0]
    loc = (
        position = (0.0, 0.0, 0.0),
        heading = 0.0,
        velocity = (1.0, 0.0, 0.0)
    )
    obstacles = []
    steer, throttle, active = plan_motion(loc, obstacles, path)
    @test active
    @test throttle > 0.0
    @test abs(steer) < 0.2

    ## === Test 2: Obstacle Directly Ahead === ##
    obstacles = [(2.0, 0.0)]
    steer, throttle, active = plan_motion(loc, obstacles, path)
    @test active
    @test isapprox(throttle, 0.0, atol=0.01)

    ## === Test 3: Obstacle to the Side (should not stop) === ##
    obstacles = [(0.0, 4.0)]  # 4m to the side
    steer, throttle, active = plan_motion(loc, obstacles, path)
    @test active
    @test throttle > 0.0

    ## === Test 4: Curved Path Following === ##
    curve_path = [SVector(x, 0.5x^2) for x in 0.0:0.5:10.0]
    loc = (
        position = (1.0, 0.5, 0.0),
        heading = 0.3,
        velocity = (2.0, 0.0, 0.0)
    )
    steer, throttle, active = plan_motion(loc, [], curve_path)
    @test active
    @test throttle > 0.0
    @test abs(steer) > 0.05  # should turn

    ## === Test 5: No Lookahead Point Found === ##
    bad_path = [SVector(x, x^2 + 20.0) for x in 0.0:1.0:5.0]
    loc = (
        position = (0.0, 0.0, 0.0),
        heading = 0.0,
        velocity = (0.0, 0.0, 0.0)
    )
    steer, throttle, active = plan_motion(loc, [], bad_path)
    @test active
    @test isapprox(throttle, 0.0, atol=0.01)
end
