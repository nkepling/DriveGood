mutable struct Track
    id::Int                     # unique track id
    x ::MVector{4,Float64}      # EKF state  [x y v θ]
    P ::SMatrix{4,4,Float64}    # EKF covariance
    last_update::Float64        # last time we updated this track
end

function fuse_detections(dets::Vector{SVector{2,Float64}}, threshold::Float64)
    fused = SVector{2,Float64}[]
    used = falses(length(dets))
    for i in eachindex(dets)
        if used[i]; continue; end
        cluster = [dets[i]]
        used[i] = true
        for j in (i+1):length(dets)
            if !used[j] && norm(dets[i] - dets[j]) < threshold
                push!(cluster, dets[j])
                used[j] = true
            end
        end
        # average cluster
        push!(fused, sum(cluster) / length(cluster))
    end
    return fused
end

function assign_detections(tracks::Vector{Track}, detections::Vector{SVector{2,Float64}}, max_dist)
    M, N = length(detections), length(tracks)
    assignments = Vector{Tuple{Int,Int}}()
    used_tracks = falses(N)
    used_dets   = falses(M)

    for di in 1:M
        best_t, best_d = 0, max_dist
        for (ti, trk) in enumerate(tracks)
            used_tracks[ti] && continue                # already taken
            d = norm(detections[di] - SVector(trk.x[1], trk.x[2]))
            if d < best_d
                best_d, best_t = d, ti
            end
        end
        if best_t > 0
            push!(assignments, (di, best_t))
            used_dets[di]   = true
            used_tracks[best_t] = true
        end
    end
    unassigned_dets   = findall(!, used_dets)
    unassigned_tracks = findall(!, used_tracks)
    return assignments, unassigned_dets, unassigned_tracks
end


struct EstimatedVehicleState
    object_id::Int
    timestamp::Float64
    pos_x::Float64 #in global coordintes
    pos_y::Float64
    pos_z::Float64
    θ::Float64
    v::Float64
    h::Float64
    l::Float64
    w::Float64
end

struct MyPerceptionType
    time::Float64
    estimated_states::Vector{EstimatedVehicleState}
end

function get_transforms(orientation_quaternion, ego_position)
    T_body_cam1 = VehicleSim.get_cam_transform(1)
    T_body_cam2 = VehicleSim.get_cam_transform(2)
    T_cam_camrot = VehicleSim.get_rotated_camera_transform()

    T_body_camrot1 = VehicleSim.multiply_transforms(T_body_cam1, T_cam_camrot)
    T_body_camrot2 = VehicleSim.multiply_transforms(T_body_cam2, T_cam_camrot)
    T_world_body = VehicleSim.get_body_transform(orientation_quaternion, ego_position)
    T_world_camrot1 = VehicleSim.multiply_transforms(T_world_body, T_body_camrot1)
    T_world_camrot2 = VehicleSim.multiply_transforms(T_world_body, T_body_camrot2)

    return (T_world_camrot1, T_world_camrot2)
end

const H_meas = @SMatrix [1.0 0 0 0;
                         0   1 0 0]



function ekf_predict!(x::MVector{4,Float64}, P::SMatrix{4,4,Float64}, ∆t::Float64, Q::SMatrix{4,4,Float64})
    v, θ = x[3], x[4]
    dx   = v * cos(θ) * ∆t
    dy   = v * sin(θ) * ∆t
    x[1] += dx
    x[2] += dy

    F = @SMatrix [1  0  cos(θ)*∆t   -v*sin(θ)*∆t;
                  0  1  sin(θ)*∆t    v*cos(θ)*∆t;
                  0  0  1            0;
                  0  0  0            1]

    return F*P*F' + Q
end

function ekf_update!(x::MVector{4,Float64}, P::SMatrix{4,4,Float64}, z::SVector{2,Float64}, R::SMatrix{2,2,Float64})
    y  = z - H_meas*x
    S  = H_meas*P*H_meas' + R
    K  = P*H_meas'*inv(S)
    x .= x + K*y

    I4 = @SMatrix [1.0 0 0 0;
                   0 1.0 0 0;
                   0 0 1.0 0;
                   0 0 0 1.0]

    return (I4 - K*H_meas)*P
end

function perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    tracks = Track[]
    next_id = 1
    delete_after = 1.0          # seconds with no update → drop track

    while true
        fresh_cam_meas = []

        @info "Grabing Measurements"
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end
        @info "Done Grabing Measurements"
        # TODO: fix this
        loc = fetch(localization_state_channel)
        # ego_position =loc.position
        ego_position =[loc.lat,loc.long]
        orientation_quaternion =loc.orientation

        T_wc1, T_wc2 = get_transforms(orientation_quaternion, ego_position)
        #cooudl the orientation_quaternion and ego_position be wrong?

        ################### DEBUG ##########
        # gt_meas = []
        # meas = fetch(gt_channel)
        # push!(gt_meas,meas)

        # T_c1w = VehicleSim.invert_transform(T_wc1)
        # T_c2w = VehicleSim.invert_transform(T_wc2)

        # gt_meas = []

        # while isready(gt_channel)
        #     push!(gt_meas, take!(gt_channel))
        # end

        # for m in gt_meas
        #     put!(gt_channel, m)
        # end

        # c1_gt_list = []
        # c2_gt_list = []

        # for m in gt_meas
        #     pos = m.position
        #     pos_world = SVector{4}(pos[1],pos[2],pos[3],1.0)
        #     push!(c1_gt_list,T_c1w*pos_world)
        #     push!(c2_gt_list,T_c2w*pos_world)
        # end
        # ## DEBUG ##

        # imgs = []

        # for m in c1_gt_list

        #     #img plane coords
        #     x = 0.64 * m[1]/68.69454692113314
        #     y = 0.64 * m[2]/68.69454692113314

        #     # to pixels

        #     push!(imgs,(x,y))
        # end

        # sanity_check =  [1.7002153478402988, 1.7048757102432166, 68.69454692113314] #gt in camerframe

        ############ DEBUG ########################################


        #estimated_states = process_camera_data(fresh_cam_meas,loc)

        # convert 
              # ── convert bboxes → world‑frame (x,y) detections ──────
        det_xy = SVector{2,Float64}[]
        det_z  = Float64[]
        cam_id_list = Int[]

        estimated_states = process_camera_data(fresh_cam_meas, loc)
        
        for (cid, pos_cam, _) in estimated_states
            pos_cam4 = SVector{4}(pos_cam[1], pos_cam[2], pos_cam[3], 1.0)
            # pos_cam4 = SVector{4}(pos_cam[1], pos_cam[2], 71, 1.0)
            T = cid == 1 ? T_wc1 : T_wc2
            pw = T * pos_cam4
            push!(det_xy, SVector(pw[1], pw[2]))
            push!(det_z,  pw[3])
            push!(cam_id_list, cid)
        end

        # ── EKF predict all tracks to current time ─────────────
        for trk in tracks
            Δt = loc.time - trk.last_update
            Q  = @SMatrix [0.04 0 0 0; 0 0.04 0 0; 0 0 0.25 0; 0 0 0 0.003]
            trk.P = ekf_predict!(trk.x, trk.P, Δt, Q)
        end

      # ── Assign detections to existing tracks ───────────────
        max_track_number = length(fresh_cam_meas[end].bounding_boxes)

        # if num_tracks < max_track_number then make a track for each new bounding_boxes untile we have max_track_number
        # if num_tracks > max_track_number then assign detections to existing tracks. then delete tracks in order of least number of updates until we have max_track number
        # if num_tracks == max_track_number then assign all detections to one of the tracks
        assign, un_det, un_trk = assign_detections(tracks, det_xy, 3.0)

        track_update_count = zeros(max_track_number)
        R = @SMatrix [1.0 0.0; 0.0 1.0]
        for (di, ti) in assign
            z = det_xy[di]
            trk = tracks[ti]
            trk.P = ekf_update!(trk.x, trk.P, z, R)
            trk.last_update = loc.time
            track_update_count[ti]+=1
        end

        cur_track_num = length(tracks)

        if cur_track_num < max_track_number
            for di in un_det
                if length(tracks) >= max_track_number
                    break
                end
                z = det_xy[di]
                x0 = @MVector [z[1], z[2], 0.0, 0.0]
                P0 = @SMatrix [4 0 0 0; 0 4 0 0; 0 0 4 0; 0 0 0 π^2]
                push!(tracks, Track(next_id, x0, P0, loc.time))
                next_id += 1
            end
        elseif cur_track_num > max_track_number
            sort!(tracks, by = t -> track_update_count[t.id])
            while length(tracks) > max_track_number
                popfirst!(tracks)
            end
        end

        # Re-assign unmatched detections if any are left
        if length(tracks) == max_track_number && !isempty(un_det)
            assign_left, _, _ = assign_detections(tracks, det_xy[un_det], 3.0)
            for (di_rel, ti) in assign_left
                di = un_det[di_rel]
                z = det_xy[di]
                trk = tracks[ti]
                trk.P = ekf_update!(trk.x, trk.P, z, R)
                trk.last_update = loc.time
            end
        end


        # ── Build output message ───────────────────────────────
        est_vec = EstimatedVehicleState[]
        for trk in tracks
            push!(est_vec, EstimatedVehicleState(trk.id, loc.time, trk.x[1], trk.x[2], 0.0, trk.x[4], trk.x[3], 0,0,0))
        end
        put!(perception_state_channel, MyPerceptionType(loc.time, est_vec))

        if fetch(shutdown_channel) == true
            break
        end
    end
end

function process_camera_data(fresh_cam_meas,loc)
    time_thresh = 0.8
    all_bbs = []

    for m in fresh_cam_meas
        time_diff = abs(loc.time - m.time)
        if time_diff <= time_thresh
            @info "Keeping camera_id=$(m.camera_id) with $(length(m.bounding_boxes)) bboxes"
            for bb in m.bounding_boxes
                push!(all_bbs, (m.camera_id, bb))  
            end
        else
            @warn "Skipping meas with time_diff=$time_diff"
        end
    end
    
    f = fresh_cam_meas[end].focal_length
    pix_len = fresh_cam_meas[end].pixel_length
    im_width = fresh_cam_meas[end].image_width
    im_height = fresh_cam_meas[end].image_height

    known_vehicle_height = 5.3

    est_states = Vector{Tuple{Int, SVector{3,Float64}, Float64}}()

    for (camera_id, bb) in all_bbs
        # x_min, y_min, x_max, y_max = bb
        top, left, bottom, right = bb
        x_min = left
        x_max = right
        y_min = top
        y_max = bottom
  
        bbox_px_width = x_max - x_min
        bbox_px_height = y_max - y_min

        #im center in pixels
        cx, cy = im_width / 2, im_height / 2

        #bb center in pixels
        u = (x_min + x_max) / 2.0
        v = (y_min + y_max) / 2.0

        camera_pitch = 0.02   # rad
        camera_height = 2.4   # meters


        # angle # a crude guess
        θ_img = atan((v - cy) * pix_len / f)
        Z_cam = (known_vehicle_height-2.4) / (cos(pi/2 - (0.02 + θ_img))) * 1.18 # this is just an emperical scaling factor

        if Z_cam < 0 
            Z_cam = -Z_cam
        end

        if camera_id == 1
            x_img = (u - cx) * pix_len
        else
            x_img = ( cx- u) * pix_len
        end
        y_img = (v - cy) * pix_len
        @info "camera_id: $camera_id"
        @info "x_img: $x_img"
        @info "y_img: $y_img"

        if camera_id == 1
            X_cam = -x_img * Z_cam / f
        else
            X_cam = x_img * Z_cam / f
        end
        
        X_cam =  x_img * Z_cam / f          
        Y_cam = y_img * Z_cam / f          
            

        push!(est_states, (camera_id, SVector{3,Float64}(X_cam, Y_cam, Z_cam), Z_cam))
    end

    return est_states
end
