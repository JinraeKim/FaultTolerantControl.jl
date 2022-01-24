"""
Get the smallest second-order mode (ssom) for linear time-invariant (LTI) systems.

# Refs
[1] N. E. Wu, K. Zhou, and G. Salomon, “Control Reconfigurability of Linear Time-Invariant Systems,” p. 5, 2000.
"""
function ssom(A, B, C, D=zeros(size(C)[1], size(B)[2]))
    # Note: ss, gram exported from `ControlSystems.jl`
    if all(real.(eigvals(A)) .< 0)  # Hurwitz matrix
        sys = ss(A, B, C, D)
        Wc = gram(sys, :c)
        Wo = gram(sys, :o)
        # Hankel singular value (HSV)
        # TODO: HSV is only valid for stable `sys`; otherwise, `gram` does not work.
        return hsv_min = Wc*Wo |> LinearAlgebra.eigvals |> minimum |> sqrt
    else
        error("TODO: ssom for unstable systems has not been developed yet.")
        # TODO: [1, Eqs. (7-8)]
    end
end
