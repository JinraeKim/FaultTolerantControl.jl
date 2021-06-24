using Test


@testset "control_systems" begin
    for f in readdir(@__DIR__)
        if f != splitdir(@__FILE__)[2]
            include(f)
        end
    end
end
