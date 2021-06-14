function State(
        multicopter::MulticopterEnv,
        fdi::DelayFDI,
        faults::Vector{AbstractFault},
    )
    return function (; args_multicopter=())
        x0_multicopter = State(multicopter)(args_multicopter...)
    end
end

function Setup(
        multicopter::MulticopterEnv,
        fdi::DelayFDI,
        faults::Vector{AbstractFault},
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
    (; Λ_func=Λ_func, Λ̂_func=Λ̂_func)
end

function Dynamics!(
        multicopter::MulticopterEnv,
        fdi::DelayFDI,
        faults::Vector{AbstractFault},
    )
    setup_data = Setup(multicopter, fdi, faults)
    @unpack Λ_func, Λ̂_func = setup_data
    return function (dX, X, p, t; u)
        Λ = Λ_func(t)
        Λ̂ = Λ̂_func(t)
        Dynamics!(multicopter)(dX, X, (), t; u=u, Λ=Λ)
    end
end

function DatumFormat(
        multicopter::MulticopterEnv,
        fdi::DelayFDI,
        faults::Vector{AbstractFault},
    )
    setup_data = Setup(multicopter, fdi, faults)
    @unpack Λ_func, Λ̂_func = setup_data
    return function (_X, t, integrator)
        X = copy(_X)
        Λ = Λ_func(t)
        Λ̂ = Λ̂_func(t)
        (; state=X, Λ=Λ, Λ̂=Λ̂)
    end
end