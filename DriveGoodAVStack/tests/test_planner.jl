using Test
using StaticArrays

include("../src/utils/geometry.jl")
include("../src/planner/pure_pursuit.jl")
include("../src/planner/planner.jl")

using .GeometryUtils
using .PurePursuit
using .Planner

@testset "Planner and Pure Pursuit Logic" begin

    loc_base = (
        position = (20.0, 5.0, 0.0),
        heading = 0.0,
        velocity = (2.0, 0.0, 0.0)
    )

    rich_path = [
        (SVector(10.0, 0.0), false, false),
        (SVector(20.0, 5.0), true, false),   # 🛑 stop
        (SVector(30.0, 10.0), false, false),
        (SVector(40.0, 15.0), false, true)   # 🅿️ pullout
    ]

    # 🧪 Test: Obstacle directly ahead
    state, stop_timer, stopped_at = :DRIVING, 0.0, time()
    obstacle = [(21.0, 5.0)]
    steer, throttle, active, _, _, _ = Planner.plan_motion(loc_base, obstacle, rich_path, state, stop_timer, stopped_at)
    @test throttle == 0.0
    @test steer == 0.0

    # 🧪 Test: Stop sign reached, must stop
    loc_at_stop = merge(loc_base, (; position=(19.8, 5.0)))
    state, stop_timer, stopped_at = :DRIVING, 0.0, time()
    steer, throttle, active, new_state, _, _ = Planner.plan_motion(loc_at_stop, [], rich_path, state, stop_timer, stopped_at)
    @test throttle == 0.0
    @test new_state == :WAITING

    # 🧪 Test: Pullout reached, should stop permanently
    loc_at_pullout = merge(loc_base, (; position=(40.0, 15.0)))
    state, stop_timer, stopped_at = :DRIVING, 0.0, time()
    steer, throttle, active, _, _, _ = Planner.plan_motion(loc_at_pullout, [], rich_path, state, stop_timer, stopped_at)
    @test throttle == 0.0
    @test active == false

    # 🧪 Test: Normal driving
    loc_drive = merge(loc_base, (; position=(15.0, 2.0)))
    state, stop_timer, stopped_at = :DRIVING, 0.0, time()
    steer, throttle, active, _, _, _ = Planner.plan_motion(loc_drive, [], rich_path, state, stop_timer, stopped_at)
    @test throttle > 0.0
    @test active == true

    # 🧪 Test: Stop sign wait timer (still waiting)
    loc_stopped = merge(loc_base, (; position=(20.0, 5.0), velocity=(0.0, 0.0, 0.0)))
    state = :WAITING
    stop_timer = 0.0
    stopped_at = time()
    steer, throttle, active, state, stop_timer, stopped_at = Planner.plan_motion(loc_stopped, [], rich_path, state, stop_timer, stopped_at)
    sleep(1.0)
    steer, throttle, active, state, stop_timer, stopped_at = Planner.plan_motion(loc_stopped, [], rich_path, state, stop_timer, stopped_at)
    @test state == :WAITING
    @test throttle == 0.0

    # 🧪 Test: Resume after 3 seconds
    sleep(2.1)
    steer, throttle, active, state, stop_timer, stopped_at = Planner.plan_motion(loc_stopped, [], rich_path, state, stop_timer, stopped_at)
    @test state == :DRIVING
    @test throttle > 0.0
end
