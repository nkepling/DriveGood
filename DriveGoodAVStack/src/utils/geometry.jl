function find_lookahead_point(path::Vector{SVector{2,Float64}}, position::SVector{2,Float64}, lookahead::Float64, facing::SVector{2,Float64})
    for i in 1:length(path)-1
        p1, p2 = path[i], path[i+1]
        seg = p2 - p1
        d = p1 - position

        a = dot(seg, seg)
        b = 2 * dot(d, seg)
        c = dot(d, d) - lookahead^2

        Δ = b^2 - 4a*c
        if Δ < 0
            continue
        end

        sqrtΔ = sqrt(Δ)
        for sign in (-1, 1)
            t = (-b + sign * sqrtΔ) / (2a)
            if 0.0 ≤ t ≤ 1.0
                pt = p1 + t * seg
                if dot(pt - position, facing) > 0
                    return pt
                end
            end
        end
    end

    return nothing
end
