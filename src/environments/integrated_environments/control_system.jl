abstract type AbstractControlSystem <: AbstractEnv end

struct BacksteppingConrtol_PseudoInverseCA_ControlSystem <: AbstractControlSystem
    controller::BacksteppingPositionControllerEnv
    allocator::AbstractAllocator
end

function State(cs::BacksteppingConrtol_PseudoInverseCA_ControlSystem)
    @unpack controller, allocator = cs
    return function (pos0, m, g)
        x_controller = State(controller)(pos0, m, g)
        ComponentArray(controller=x_controller)
    end
end

function Command(cs::BacksteppingConrtol_PseudoInverseCA_ControlSystem)
    @unpack controller, allocator = cs
    return function (p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g, Λ̂)
        νd, Ṫd = command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
        u_cmd = allocator(νd, Λ̂)
        ComponentArray(νd=νd, Ṫd=Ṫd, u_cmd=u_cmd)
    end
end

function Dynamics!(cs::BacksteppingConrtol_PseudoInverseCA_ControlSystem)
    @unpack controller = cs
    @Loggable function dynamics!(dx, x, p, t; pos_cmd=nothing, Ṫd)
        @nested_log :controller Dynamics!(controller)(dx.controller, x.controller, p, t; pos_cmd=pos_cmd, Ṫd=Ṫd)
    end
end
