"""
FDI with simple time delay.
Λ_func: function of `t`.
"""
struct DelayFDI <: AbstractFDI
    τ
    function DelayFDI(τ)
        @assert τ > 0
        new(τ)
    end
end

function Estimate(fdi::DelayFDI, Λ_func)
    @unpack τ = fdi
    return function (t)
        Λ̂ = Λ_func(t - τ)
    end
end

# function State(fdi::DelayFDI)
#     return function (dim_input::Int)
#         ComponentArray(Λ̂=ones(dim_input) |> Diagonal |> Matrix)
#     end
# end

# """
# Λ: effectiveness matrix
# # Notes
# For delayed FDI (DelayFDI),
# the function of effectiveness matrix should be known.
# For the explanation of the below implementation,
# see https://discourse.julialang.org/t/differentialequations-jl-discrete-delay-problem/62829/10?u=ihany.
# # NOTICE
# For some technical issue,
# the matrix-form state is dealt with by ComponentArray (default setting).
# """
# function Dynamics!(fdi::DelayFDI, Λ_func)
#     @unpack τ = fdi
#     dim_input = Λ_func(0.0) |> length  # auxiliary evaluation
#     dynamics! = function (dX, X, p, t)
#         dX.Λ̂ .= X.Λ̂ - Λ_func(t - τ)
#         nothing
#     end
#     ODEFunction(dynamics!, mass_matrix=zeros(dim_input) |> Diagonal |> Matrix)
# end
