"""
Low pass filter (LPF) -like FDI.
"""
struct LPFFDI <: AbstractFDI
    τ
    function LPFFDI(τ)
        @assert τ > 0
        new(τ)
    end
end

function State(fdi::LPFFDI)
    return function (dim_input::Int)
        Λ̂ = Diagonal(ones(dim_input))
    end
end

"""
Λ: effectiveness matrix
"""
function Dynamics!(fdi::LPFFDI)
    @unpack τ = fdi
    return function (dΛ̂, Λ̂, p, t; Λ)
        dΛ̂ .= (Λ - Λ̂) / τ
    end
end
