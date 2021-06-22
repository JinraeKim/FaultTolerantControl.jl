abstract type AbstractPlant <: AbstractEnv end

struct DelayFDI_Plant <: AbstractPlant
    multicopter::MulticopterEnv
    fdi::DelayFDI
    faults::Vector{AbstractFault}
end


function State(env::DelayFDI_Plant)
    @unpack multicopter, fdi, faults = env
    return function (; args_multicopter=())
        x_multicopter = State(multicopter)(args_multicopter...)
        ComponentArray(multicopter=x_multicopter)
    end
end

function EffectivenessMatrixFunction(env::DelayFDI_Plant)
    @unpack multicopter, fdi, faults = env
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

function Dynamics!(env::DelayFDI_Plant)
    @unpack multicopter, fdi, faults = env
    effectiveness_matrix_functions = EffectivenessMatrixFunction(env)
    @unpack Λ_func, Λ̂_func = effectiveness_matrix_functions
    @Loggable function dynamics!(dX, X, p, t; u)
        Λ = Λ_func(t)
        @nested_log :FDI Λ̂ = Λ̂_func(t)
        @nested_log Dynamics!(multicopter)(dX.multicopter, X.multicopter, (), t; u=u, Λ=Λ)
    end
end
