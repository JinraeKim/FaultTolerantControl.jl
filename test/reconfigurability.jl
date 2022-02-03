using FaultTolerantControl
using LinearAlgebra
using Test


@testset "reconfigurability for linear system" begin
    # [1, Section 3, example 1]
    # # Refs
    # [1] N. E. Wu, K. Zhou, and G. Salomon, “Control Reconfigurability of Linear Time-Invariant Systems,” p. 5, 2000.
    A = [
         -0.0226 -36.6 -18.9 -32.1;
         0 -1.9 0.983 0;
         0.0123 -11.7 -2.63 0;
         0 0 1 0;
        ]
    B = [
         0 0;
         -0.414 0;
         -77.8 22.4;
         0 0;
        ]
    C = [
         0 5.73 0 0;
         0 0 0 5.73;
        ]
    D = [
         0 0;
         0 0;
        ]
    n = size(B)[1]
    ssom_min = ssom(A, B, C, D)
    @test ssom_min >= 10 && ssom_min < 12
end

@testset "reconfigurability" begin
    # # Refs
    # [2] https://github.com/gramian/emgr
    A = -Matrix(I, 4, 4)
    B = [0.0 1.0 0.0 1.0]'
    C = [0.0 0.0 1.0 1.0]

    f(x, u, p, t) = A*x + B*u .+ p
    g(x, u, p, t) = C*x

    # System dimension
    n = 4
    m = 1
    l = 1

    x0 = [0.0, 0.0, 0.0, 0.0]
    u0 = [0.0]

    Wc = empirical_gramian(f, g, m, n, l; opt=:c, dt=0.01, tf=1.0, pr=zeros(4, 1), xs=x0, us=u0)
    Wo = empirical_gramian(f, g, m, n, l; opt=:o, dt=0.01, tf=1.0, pr=zeros(4, 1), xs=x0, us=u0)
	minHSV = min_HSV(Wc, Wo)
	@show Wc Wo minHSV
    @test Wc != nothing && Wo != nothing
end
