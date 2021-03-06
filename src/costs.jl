abstract type AbstractCostFunctional end


"""
Default weight: 1 for position, (1/π^2) for angular velocity (1 m ≈ 180 deg)
# Notes
- Q_p: weight matrix for running cost (position)
- Q_ω: weight matrix for running cost (angular velocity)
- F_p: weight matrix for terminal cost (position)
- F_ω: weight matrix for terminal cost (angular velocity)
"""
struct PositionAngularVelocityCostFunctional <: AbstractCostFunctional
    Q_p::AbstractMatrix
    Q_ω::AbstractMatrix
    F_p::AbstractMatrix
    F_ω::AbstractMatrix
    function PositionAngularVelocityCostFunctional(
            Q_p=Matrix(I, 3, 3), Q_ω=Matrix(I, 3, 3),
            F_p=(1/π^2)*Matrix(I, 3, 3), F_ω=(1/π^2)*Matrix(I, 3, 3),
        )
        new(Q_p, Q_ω, F_p, F_ω)
    end
end

"""
ts: an array of time
e_ps: an array of position error
e_ωs: an array of angular velocity error
"""
function cost(cf::PositionAngularVelocityCostFunctional,
        ts::AbstractVector, e_ps::AbstractVector, e_ωs::AbstractVector,
    )
    @unpack Q_p, Q_ω, F_p, F_ω = cf
    running_costs = zip(e_ps, e_ωs) |> MapSplat((e_p, e_ω) -> e_p'*Q_p*e_p + e_ω'*Q_ω*e_ω) |> collect
    e_p_f = e_ps[end]
    e_ω_f = e_ωs[end]
    terminal_cost = e_p_f'*F_p*e_p_f + e_ω_f'*F_ω*e_ω_f
    integrate(ts, running_costs) + terminal_cost
end
