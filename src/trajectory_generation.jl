abstract type AbstractTrajectory end

"""
Bezier curve
θ: a vector whose length is (N+1); each element can also be vector,
e.g., θ = [[1, 2, 3], [2, 3, 4]]
"""
struct Bezier <: AbstractTrajectory
    θ::AbstractVector
    t0::Real
    tf::Real
end

function (bezier::Bezier)(t::Real)
    @unpack t0, tf, θ = bezier
    N = length(θ) -1
    @assert t <= tf && t >= t0
    s = (t-t0)/(tf-t0)
    basis = 0:N |> Map(i -> binomial(N, i) * s^i * (1-s)^(N-i)) |> collect
    basis .* θ |> sum  # sum of N_C_i * s^(N-i) * (1-s)^i * θ_i
end
