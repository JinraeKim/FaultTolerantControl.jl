# TODO!!!
abstract type AbstractFeedbackSystem <: AbstractEnv end

struct DelayFDI_Plant_BacksteppingControl_PseudoInverseCA_FeedbackSystem <: AbstractFeedbackSystem
    plant::DelayFDI_Plant
    control_system::BacksteppingConrtol_PseudoInverseCA_ControlSystem
end

"""
MulticopterEnv + BacksteppingPositionControllerEnv
"""
function State(env::DelayFDI_Plant_BacksteppingControl_PseudoInverseCA_FeedbackSystem)
    @unpack plant, control_system = env
    @unpack multicopter = plant
    @unpack controller = control_system
    @unpack m, g = multicopter
    return function (; args_multicopter=())
        x_plant = State(plant)(args_multicopter...)
        pos0 = copy(x_plant.multicopter.p)
        x_cs = State(control_system)(pos0, m, g)
        ComponentArray(plant=x_plant, control_system=x_cs)
    end
end

function Dynamics!(env::DelayFDI_Plant_BacksteppingControl_PseudoInverseCA_FeedbackSystem)
    @unpack plant, control_system = env
    effectiveness_matrix_functions = EffectivenessMatrixFunction(plant)
    @unpack Λ̂_func = effectiveness_matrix_functions
    @unpack controller, allocator = control_system
    @unpack multicopter = plant
    @unpack m, J, g = multicopter
    return function (dx, x, p, t; pos_cmd=nothing)
        Λ̂ = Λ̂_func(t)
        @unpack p, v, R, ω = x.plant.multicopter
        @unpack ref_model, Td = x.control_system.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        command_info = Command(control_system)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g, Λ̂)
        @unpack νd, Ṫd, u_cmd = command_info
        Dynamics!(plant)(dx.plant, x.plant, (), t; u=u_cmd)
        Dynamics!(control_system)(dx.control_system, x.control_system, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        nothing
    end
end

function DatumFormat(env::DelayFDI_Plant_BacksteppingControl_PseudoInverseCA_FeedbackSystem)
    @unpack plant, control_system = env
    @unpack multicopter, fdi, faults = plant
    @unpack controller, allocator = control_system
    effectiveness_matrix_functions = EffectivenessMatrixFunction(plant)
    @unpack Λ_func, Λ̂_func = effectiveness_matrix_functions
    @unpack m, J, g = multicopter
    return function (_x, t, integrator)
        x = copy(_x)
        Λ = Λ_func(t)
        Λ̂ = Λ̂_func(t)
        @unpack p, v, R, ω = x.plant.multicopter
        @unpack ref_model, Td = x.control_system.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        command_info = Command(control_system)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g, Λ̂)
        @unpack νd, Ṫd, u_cmd = command_info
        u = FS.saturate(multicopter, u_cmd)
        (; state=x, Λ=Λ, Λ̂=Λ̂, pos=p, u_cmd=u_cmd, u=u)
    end
end
