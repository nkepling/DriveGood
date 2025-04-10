function logger(info_channel, shutdown_channel)
    #= Logger for async debug purposes
    =#

    while true
        fetch!(shutdown_channel) && break

        while isready(info_channel)
            meas = take!(info_channel)
            if meas.msg_type == "info"

                info_str = meas.str
                @printf("%s \u1b[0K \n", info_str)
            end
        end
        flush(stdout)
    end
end
