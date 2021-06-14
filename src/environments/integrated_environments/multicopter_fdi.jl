function State(
        multicopter::MulticopterEnv,
        fdi::DelayFDI,
        faults::Vector{AbstractFault}=AbstractFault[],
    )
    return function (; args_multicopter=())
        x0_multicopter = State(multicopter)(args_multicopter...)
        ComponentArray(multicopter=x0_multicopter)
    end
end

function Dynamics!(
        multicopter::MulticopterEnv,
        fdi::DelayFDI,
        faults::Vector{AbstractFault}=AbstractFault[],
    )
    @unpack dim_input = multicopter
    # actuator faults
    actuator_faults = faults |> Filter(fault -> typeof(fault) <: AbstractActuatorFault) |> collect
    affect_actuator! = Affect!(AbstractActuatorFault[actuator_faults...])
    # effectiveness matrix
    Λ_func = function (t)
        Λ = ones(dim_input) |> Diagonal
        affect_actuator!(Λ, t)
        Λ
    end
    # delayed effectiveness matrix estimation
    Λ̂_func = Estimate(fdi, Λ_func)
    return function (dX, X, p, t; u)
        Λ = Λ_func(t)
        Λ̂ = Λ̂_func(t)
        Dynamics!(multicopter)(dX.multicopter, X.multicopter, (), t; u=u, Λ=Λ)
    end
end
