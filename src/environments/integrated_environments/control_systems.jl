abstract type AbstractControlSystem <: AbstractEnv end


# BacksteppingControl_StaticAllocator_ControlSystem
struct BacksteppingControl_StaticAllocator_ControlSystem <: AbstractControlSystem
    controller::BacksteppingPositionControllerEnv
    allocator::StaticAllocator
end

function State(cs::BacksteppingControl_StaticAllocator_ControlSystem)
    @unpack controller, allocator = cs
    return function (pos0, m, g)
        x_controller = State(controller)(pos0, m, g)
        ComponentArray(controller=x_controller)
    end
end

function Command(cs::BacksteppingControl_StaticAllocator_ControlSystem)
    @unpack controller, allocator = cs
    return function (p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g, Λ̂)
        νd, Ṫd, _... = Command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
        u_cmd = allocator(νd, Λ̂)
        ComponentArray(νd=νd, Ṫd=Ṫd, u_cmd=u_cmd)
    end
end

function Dynamics!(cs::BacksteppingControl_StaticAllocator_ControlSystem)
    @unpack controller = cs
    @Loggable function dynamics!(dx, x, p, t; pos_cmd=nothing, Ṫd)
        @nested_log :controller Dynamics!(controller)(dx.controller, x.controller, p, t; pos_cmd=pos_cmd, Ṫd=Ṫd)
    end
end


# BacksteppingControl_AdaptiveAllocator_ControlSystem
struct BacksteppingControl_AdaptiveAllocator_ControlSystem <: AbstractControlSystem
    controller::BacksteppingPositionControllerEnv
    allocator::AdaptiveAllocator
end

function State(cs::BacksteppingControl_AdaptiveAllocator_ControlSystem)
    @unpack controller, allocator = cs
    return function (pos0, m, g; Θ̂=zeros(6, 4))
        x_controller = State(controller)(pos0, m, g)
        x_allocator = State(allocator)(Θ̂)
        ComponentArray(controller=x_controller, allocator=x_allocator)
    end
end

function Command(cs::BacksteppingControl_AdaptiveAllocator_ControlSystem)
    @unpack controller, allocator = cs
    @unpack Ap, Bp, P, Kp, Kt, Kω = controller
    return function (p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g, Λ̂, Θ̂)
        νd, Ṫd, e, zB, T = Command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
        Θ̂̇ = Command(allocator)(νd, e, zB, T, Θ̂, R, J,
                               Ap, Bp, P, Kp, Kt, Kω)
        u_cmd = allocator(νd, Θ̂)
        ComponentArray(νd=νd, Ṫd=Ṫd, u_cmd=u_cmd, Θ̂̇=Θ̂̇)
    end
end

function Dynamics!(cs::BacksteppingControl_AdaptiveAllocator_ControlSystem)
    @unpack controller, allocator = cs
    @Loggable function dynamics!(dx, x, p, t; pos_cmd=nothing, Ṫd, Θ̂̇)
        @nested_log :controller Dynamics!(controller)(dx.controller, x.controller, p, t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        @nested_log :allocator Dynamics!(allocator)(dx.allocator, x.allocator, p, t; Θ̂̇=Θ̂̇)
    end
end
