using FaultTolerantControl
const FTC = FaultTolerantControl
using UnPack
using LinearAlgebra
using Test


function test()
    multicopter = LeeHexacopterEnv()
    @unpack m, g, B, u_min, u_max, dim_input = multicopter
    allocator = ConstrainedAllocator(B, u_min, u_max)
    Λ = Diagonal((ones(dim_input)))
    f = m * g
    M = zeros(3)
    ν = [f, M...]
    u = allocator(ν, Λ)
    @test ν ≈ B*Λ*u
    @test all(u .>= u_min)
    @test all(u .<= u_max)
end
