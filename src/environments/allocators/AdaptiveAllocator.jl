"""
# References
- Controller
[1] G. P. Falconi and F. Holzapfel,
“Adaptive Fault Tolerant Control Allocation for a Hexacopter System,”
Proc. Am. Control Conf., vol. 2016-July, pp. 6760–6766, 2016.
- Reference model (e.g., xd, vd, ad, ad_dot, ad_ddot)
[2] S. J. Su, Y. Y. Zhu, H. R. Wang, and C. Yun, “A Method to Construct a Reference Model for Model Reference Adaptive Control,” Adv. Mech. Eng., vol. 11, no. 11, pp. 1–9, 2019.
"""
struct AdaptiveAllocator <: AbstractEnv
    B_pinv
    γ
    function AdaptiveAllocator(B, γ=1e-1)
        B_pinv = pinv(B)
        new(B_pinv, γ)
    end
end

function State(allocator::AdaptiveAllocator)
    return function (Θ̂=zeros(6, 4))
        Θ̂
    end
end

function Dynamics!(allocator::AdaptiveAllocator)
    @Loggable function dynamics!(dΘ̂, Θ̂, p, t; Θ̂̇)
        @log Θ̂
        dΘ̂ .= Θ̂̇
    end
end

"""
Several functions are exported from `utils.jl`, e.g., T_u_inv(T).
"""
function Command(allocator::AdaptiveAllocator)
    error("TODO")
    @unpack B, γ = allocator
    return function (νd, e, zB, R, J,
                     Ap, Bp, P, Kp, Kt, Kω)
        P̄ = [          P         zeros(6, 6);
             zeros(6, 6) 0.5*Matrix(I, 6, 6)]
        θ_ėp = Bp
        θ_u̇1 = Kp * θ_ėp
        θ_ėt = θ_u̇1
        θ_ëp = Ap * θ_ėp + Bp * θ_ėt
        θ_ü1 = Kp * θ_ëp
        θ_u̇2 = T_u_inv(T) * R * (2*Bp' * P * θ_ėp
                                 + θ_ü1 + Kt * θ_ėt)
        θ_ω̇d = [1 0 0;
                0 1 0;
                0 0 0] * θ_u̇2
        B̄ = [-(θ_ėp*zB) zeros(6, 3);
             -(θ_u̇1*zB) zeros(3, 3);
             -(θ_ω̇d*zB) inv(J)]
        Θ̂̇ = γ * proj_R(
                       Θ̂,
                       (νd * e' * P̄ * B̄ * B)'
                      )
    end
end

function Proj_R(C, Y)
    proj_R = zeros(size(Y))
    for i in 1:size(Y)[1]
        ci = C[i, :]
        yi = Y[i, :]
        proj_R[i, :] = Proj(ci, yi)
    end
    proj_R
end

function Proj(θ, y; ϵ=1e-2, θ_max=1e5)
    f = ((1+ϵ) * norm(θ)^2 - θ_max^2) / (ϵ * θ_max^2)
    ∇f = (2*(1+ϵ) / (ϵ * θ_max^2)) * θ
    proj = nothing
    if f > 0 && ∇f' * y > 0
        ∇f_unit = ∇f / norm(∇f)
        proj = y - dot(∇f, y) * ∇f_unit
    else
        proj = y
    end
    proj
end

function (allocator::AdaptiveAllocator)(ν, Θ̂)
    @unpack B_pinv = allocator
    u = (B_pinv + Θ̂) * ν
end
