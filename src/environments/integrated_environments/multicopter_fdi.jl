"""
m: dimension of input
"""
function State(multicopter::MulticopterEnv, fdi::AbstractFDI)
    @unpack dim_input = multicopter
    return function (args_muticopter=())
        x_multicopter = State(multicopter)(args_muticopter...)
        Λ̂ = State(fdi)(dim_input)
        ComponentArray(multicopter=x_multicopter, Λ̂=Λ̂)
    end
end

function Dynamics!(multicopter::MulticopterEnv, fdi::AbstractFDI)
    return function (dX, X, p, t; u, Λ)
        Dynamics!(multicopter)(dX.multicopter, X.multicopter, (), t; u=u, Λ=Λ)
        Dynamics!(fdi)(dX.Λ̂, X.Λ̂, (), t; Λ=Λ)
    end
end
