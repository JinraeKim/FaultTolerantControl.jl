abstract type Multicopter_FDI_Faults <: AbstractEnv end

struct Multicopter_DelayFDI_Faults <: Multicopter_FDI_Faults
    multicopter::MulticopterEnv
    fdi::DelayFDI
    faults::Vector{AbstractFault}
end


function State(env::Multicopter_DelayFDI_Faults)
    @unpack multicopter, fdi, faults = env
    return function (; args_multicopter=())
        x0_multicopter = State(multicopter)(args_multicopter...)
    end
end

function EffectivenessMatrixFunction(env::Multicopter_DelayFDI_Faults)
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

function Dynamics!(env::Multicopter_DelayFDI_Faults)
    @unpack multicopter, fdi, faults = env
    effectiveness_matrix_functions = EffectivenessMatrixFunction(env)
    @unpack Λ_func, Λ̂_func = effectiveness_matrix_functions
    return function (dX, X, p, t; u)
        Λ = Λ_func(t)
        Λ̂ = Λ̂_func(t)
        Dynamics!(multicopter)(dX, X, (), t; u=u, Λ=Λ)
    end
end

function DatumFormat(env::Multicopter_DelayFDI_Faults)
    @unpack multicopter, fdi, faults = env
    effectiveness_matrix_functions = EffectivenessMatrixFunction(env)
    @unpack Λ_func, Λ̂_func = effectiveness_matrix_functions
    return function (_X, t, integrator)
        X = copy(_X)
        Λ = Λ_func(t)
        Λ̂ = Λ̂_func(t)
        (; state=X, Λ=Λ, Λ̂=Λ̂)
    end
end
