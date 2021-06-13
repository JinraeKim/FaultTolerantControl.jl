"""
FDI with simple time delay.
Λ_func: function of `t`.
"""
struct DelayFDI <: AbstractFDI
    τ
    Λ_func
    function DelayFDI(τ, Λ_func::Function)
        @assert τ > 0
        new(τ, Λ_func)
    end
end

function State(fdi::DelayFDI)
    return function (dim_input::Int)
        fill(nothing, dim_input)  # meaningless
    end
end

"""
Λ: effectiveness matrix
# Notes
For delayed FDI (DelayFDI),
the function of effectiveness matrix should be known.
For the explanation of the below implementation,
see https://discourse.julialang.org/t/differentialequations-jl-discrete-delay-problem/62829/10?u=ihany.
"""
function Dynamics!(fdi::DelayFDI)
    @unpack τ, Λ_func = fdi
    dynamics! = function (dΛ̂, Λ̂, p, t)
        dΛ̂ .= Λ̂ - Λ_func(t - τ)
        nothing
    end
    ODEFunction(dynamics!, mass_matrix=zeros(dim_input) |> Diagonal |> Matrix)
end
