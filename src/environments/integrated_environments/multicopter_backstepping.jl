# TODO!!!
abstract type Multicopter_FDI_Faults_ControlSystem <: AbstractEnv end

struct Multicopter_DelayFDI_Faults_BacksteppingControl_PseudoInverseCA <: Multicopter_FDI_Faults_ControlSystem
    multicopter_fdi_faults::Multicopter_FDI_Faults
    controller::BacksteppingPositionControllerEnv
    allocator::PseudoInverseControlAllocator
end

"""
MulticopterEnv + BacksteppingPositionControllerEnv
"""
function State(env::Multicopter_DelayFDI_Faults_BacksteppingControl_PseudoInverseCA)
    @unpack multicopter_fdi_faults, controller = env
    @unpack multicopter = multicopter_fdi_faults
    @unpack m, g = multicopter
    return function (; args_multicopter=())
        x_multicopter = State(multicopter_fdi_faults)(args_multicopter...)
        pos0 = copy(x_multicopter.p)
        x_controller = State(controller)(pos0, m, g)
        ComponentArray(multicopter=x_multicopter, controller=x_controller)
    end
end

function command(
        controller::BacksteppingPositionControllerEnv,
        allocator::PseudoInverseControlAllocator,
        p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g, Λ̂,
    )
    νd, Ṫd = command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
    u_cmd = allocator(νd, Λ̂)
    ComponentArray(νd=νd, Ṫd=Ṫd, u_cmd=u_cmd)
end

function Dynamics!(env::Multicopter_DelayFDI_Faults_BacksteppingControl_PseudoInverseCA)
    @unpack multicopter_fdi_faults, controller, allocator = env
    effectiveness_matrix_functions = EffectivenessMatrixFunction(multicopter_fdi_faults)
    @unpack Λ̂_func = effectiveness_matrix_functions
    @unpack multicopter = multicopter_fdi_faults
    @unpack m, J, g = multicopter
    return function (dx, x, p, t; pos_cmd=nothing)
        Λ̂ = Λ̂_func(t)
        @unpack p, v, R, ω = x.multicopter
        @unpack ref_model, Td = x.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        command_info = command(controller, allocator, p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g, Λ̂)
        @unpack νd, Ṫd, u_cmd = command_info
        Dynamics!(multicopter_fdi_faults)(dx.multicopter, x.multicopter, (), t; u=u_cmd)
        Dynamics!(controller)(dx.controller, x.controller, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        nothing
    end
end

function DatumFormat(env::Multicopter_DelayFDI_Faults_BacksteppingControl_PseudoInverseCA)
    @unpack multicopter_fdi_faults, controller, allocator = env
    @unpack multicopter, fdi, faults = multicopter_fdi_faults
    effectiveness_matrix_functions = EffectivenessMatrixFunction(multicopter_fdi_faults)
    @unpack Λ_func, Λ̂_func = effectiveness_matrix_functions
    @unpack m, J, g = multicopter
    return function (_x, t, integrator)
        x = copy(_x)
        pos = x.multicopter.p
        Λ = Λ_func(t)
        Λ̂ = Λ̂_func(t)
        @unpack p, v, R, ω = x.multicopter
        @unpack ref_model, Td = x.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        command_info = command(controller, allocator, p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g, Λ̂)
        @unpack νd, Ṫd, u_cmd = command_info
        u = FS.saturate(multicopter, u_cmd)
        (; state=x, Λ=Λ, Λ̂=Λ̂, pos=pos, u_cmd=u_cmd, u=u)
    end
end
