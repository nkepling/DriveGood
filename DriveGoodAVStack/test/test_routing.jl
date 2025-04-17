using Test
using JLD2
using Infiltrator
using VehicleSim
using DriveGoodAVStack

@testset "test routing" begin
    map = VehicleSim.city_map()

    starting_id = 35 # StandardSegment
    target_id = 79 # actual loading zone

    path = DriveGoodAVStack.routing(map,starting_id,target_id)
    

    @test path[1] == starting_id
    @test path[end] == target_id

    @info "$path"

    #get coordinate waypoints 
    way_points,road_ids = DriveGoodAVStack.get_way_points_from_path(path,map)

    @test !isempty(way_points)
    @test all(wp -> isa(wp, Vector{Float64}) && length(wp) == 2, way_points)



    seg = map[starting_id]
    left_a, left_b = seg.lane_boundaries[1].pt_a, seg.lane_boundaries[1].pt_b
    right_a, right_b = seg.lane_boundaries[2].pt_a, seg.lane_boundaries[2].pt_b

    # Projected start and end center points
    start_center = 0.5 .* (left_a + right_a)    
    end_center   = 0.5 .* (left_b + right_b)

    start_way_point  = way_points[1]
    @info "start_way_point $start_way_point"

    @info "expected start $start_center"

    @test isapprox(start_way_point,start_center)
  

    final_seg = map[target_id]

    left_a, left_b = final_seg.lane_boundaries[1].pt_a, final_seg.lane_boundaries[1].pt_b
    right_a, right_b = final_seg.lane_boundaries[2].pt_a, final_seg.lane_boundaries[2].pt_b

    # Projected start and end center points
    start_center = 0.5 .* (left_a + right_a)    
    end_center   = 0.5 .* (left_b + right_b)

    end_way_point  = way_points[end]

    @test isapprox(end_way_point,end_center)


    # test get poly line


    pl = DriveGoodAVStack.get_polyline(way_points)

    @test !isempty(pl.segments)

    # @info "this is the polyline $(pl.segments)

end

@testset "polyline sanity" begin
    map = VehicleSim.city_map()
    path = DriveGoodAVStack.routing(map, 35, 79)
    waypoints,road_ids = DriveGoodAVStack.get_way_points_from_path(path, map)

    pl = DriveGoodAVStack.Polyline(waypoints)

    for seg in pl.segments
        if seg isa DriveGoodAVStack.StandardSegment ||
           seg isa DriveGoodAVStack.InitialRay ||
           seg isa DriveGoodAVStack.TerminalRay

            @test all(!isnan(x) for x in seg.tangent)
            @test all(!isnan(x) for x in seg.normal)
        end
    end
end


@testset "Polyline length matches road segment ID count" begin
    map = VehicleSim.city_map()
    ego_id = 35
    target_id = 79

    waypoints, road_ids = DriveGoodAVStack.get_way_points_from_path(
        DriveGoodAVStack.routing(map, ego_id, target_id),
        map
    )

    @test length(waypoints) == length(road_ids)
end