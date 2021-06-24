"""
Constrained-optimisation-based control allocator.
"""
struct ConstrainedAllocator <: AbstractAllocator
    B
    u_min
    u_max
    u
    function ConstrainedAllocator(B, u_min, u_max; p=Inf)
        @assert size(u_min) == size(u_max)
        dim_input = length(u_min)
        u = Convex.Variable(dim_input)
        new(B, u_min, u_max, u)
    end
end

"""
# Variables
ν: virtual input
# Notes
ν = B*u where u: control input
"""
function (allocator::ConstrainedAllocator)(ν, Λ=Diagonal(ones(size(ν)));
                                           p=Inf, silent_solver=true)
    @unpack u_min, u_max, u, B = allocator
    prob = minimize(
                    norm(u, p)
                    + 1e5*norm(ν - B*Λ*u, 1)  # exact penalty method
                   )
    prob.constraints += [
                         u .>= u_min;  # min
                         u .<= u_max;  # max
                         # ν == B*Λ*u  # equality; replaced by exact penalty method
                        ]
    Convex.solve!(prob,
                  Mosek.Optimizer();
                  silent_solver=silent_solver, warmstart=true)
    u.value
end
