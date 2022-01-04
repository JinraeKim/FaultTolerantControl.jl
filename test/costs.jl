using FaultTolerantControl
using Test
using LinearAlgebra
using Transducers


@testset "costs" begin
    Q_p = Matrix(I, 3, 3)
    Q_ω = Matrix(I, 3, 3)
    F_p = Matrix(I, 3, 3)
    F_ω = Matrix(I, 3, 3)
    cf = PositionAngularVelocityCostFunctional(Q_p, Q_ω, F_p, F_ω)
    # data
    t0, tf = 0.0, 1.0
    Δt = tf - t0
    ts = t0:0.01:tf
    e_p_nom = ones(3)
    e_ω_nom = ones(3)
    e_ps = ts |> Map(t -> e_p_nom) |> collect
    e_ωs = ts |> Map(t -> e_ω_nom) |> collect
    # only test whether it works
    J = cost(cf, ts, e_ps, e_ωs)
    J ≈ e_p_nom'*Q_p*e_p_nom*Δt + e_ω_nom'*Q_ω*e_ω_nom + e_ps[end]'*F_p*e_ps[end] + e_ωs[end]'*F_ω*e_ωs[end]
end
