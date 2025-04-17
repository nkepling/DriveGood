# log measuments from server. Save to disk usign JLD2

@testset "localization test" begin

    my_buffers = load("my_buffers.jld2")
    gt_buffer = my_buffers["gt_buffer"]

    # See my client funciton in exampl_project.jl for how we can collect these buffers for testing (note the car wont be moving )
    # also call server(;measure_all=true) to enable measument streams.. defaults to no measuments.


    # some how make user EKF estimate matches measuments in gt_buffer...
    # Calculate squared distance between GT and Estimate, make sure it's below some theshold. 
    
end