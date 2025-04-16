
function routing(map,ego_road_segment_id,target_road_segment_id)
    # Dijkstras on an edge list for no assume all road segments are the same distance. 
    # Later we will compute edge weights using lane bounderies divided by speed limit.


    dist = Dict(k => Inf for k in keys(map))
    prev = Dict(k => nothing for k in keys(map))

    Q = PriorityQueue{Int,Float64}()


    dist[ego_road_segment_id] = 0

    enqueue!(Q, ego_road_segment_id, 0.0)

    while !isempty(Q)

        u,_ = dequeue!(Q) # dequeue seg id and dist from ego_road_segment_id

        if u == target_road_segment_id
            break
        end

        for v in map[u].children 
            alt = dist[u] + 1.0 # the distance here is assumed to be 1 but in reality it should probablyt be a time (Roadsegment_dist * 1/(speed_limit))
            if alt < dist[v]
                dist[v] = alt
                prev[v] = u
                enqueue!(Q,v,alt)
            end
        end
    end


    path = []
    u = target_road_segment_id
    while u !== nothing
        push!(path,u)
        u = prev[u]
    end 

    reverse!(path)
    return path
end


function construct_polyline_from_path(path::Vector{Int}, map)
    polyline = []

    for seg_id in path
        segment = map[seg_id]

        # Get the centerline points (you may need to adjust depending on your map format)
        for pt in segment.centerline
            push!(polyline, (SVector(pt[1], pt[2]), false, false))  # default: no stop/pullout
        end
    end

    # OPTIONAL: tag stop sign or pullout on last few points
    if !isempty(polyline)
        polyline[end] = (polyline[end][1], false, true)  # Mark final as pullout zone
        if length(polyline) > 2
            polyline[end - 2] = (polyline[end - 2][1], true, false)  # Mark earlier one as stop
        end
    end

    return polyline
end
