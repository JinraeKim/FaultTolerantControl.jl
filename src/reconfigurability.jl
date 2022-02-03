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

"""
    empirical_gramian()

A function for computation of controllability or observability gramian.

# References
[1] M. Tahavori and A. Hasan, “Fault recoverability for nonlinear systems with application to fault tolerant control of UAVs,” Aerosp. Sci. Technol., vol. 107, p. 106282, 2020, doi: 10.1016/j.ast.2020.106282.
[2] C. Himpe, “emgr-The empirical Gramian framework,” Algorithms, vol. 11, no. 7, pp. 1–27, 2018, doi: 10.3390/a11070091.

# Notes
- f = function of (x, u, p, t)
- g = function of (x, u, p, t)
- pr = parameter vector(s), each column i sone parameter sample
- xs = steady-state and nominal initial state x_0
- us = steady-state input
- opt = :c for controllability gramian and :o for observability gramian
"""
function empirical_gramian(f::Function, g::Function, M::Int, N::Int, L::Int; opt::Symbol, dt::Real, tf::Real, pr::Matrix, xs::Vector, us::Vector)
    @assert dt > 0 && tf > dt
    P, K = size(pr)
    ut = t -> (t <= dt) / dt
    up = t -> us
    R = L
    G(x, u, p, t) = x
    
    xm = ones(N, 1) * Matrix([-1 1])  # initial-state scales
    um = ones(M, 1) * Matrix([-1 1])  # input scales
    
    A = size(xm)[1]  # Number of total states
    C = size(um)[2]  # Number of input scales sets
    D = size(xm)[2]  # Number of state scales sets
    
    if opt === :c
        # Empirical_ctrl_gramian
        Wc = 0
        for k = 1:K
            for c = 1:C
                for m = 1:M
                    if um[m, c] != 0
                        em = zeros(M+P)
                        em[m] = um[m, c]
                        umc(t) = ut(t) .* em[1:M]
                        pmc = pr[:, k] + em[M+1:end]
                        x = ssp2(f, G, dt, tf, xs, umc, pmc)
                        x = x ./ um[m, c]
                        Wc = Wc .+ x * x'  # Question(why is this different dot(x, x')?)
                    end
                end
            end
        end
        Wc = Wc * (dt / (C * K))
    elseif opt === :o
        # Empirical_obsv_gramian
        Wo = 0
        o = zeros(R*Int(floor(tf / dt) + 1), A)
        for k = 1:K
            for d = 1:D
                for a = 1:A
                    if xm[a, d] != 0
                        en = zeros(N+P)
                        en[a] = xm[a, d]
                        xnd = xs + en[1:N]
                        pnd = pr[:, k] + en[N+1:end]
                        y = ssp2(f, g, dt, tf, xnd, up, pnd)
                        y = y ./ xm[a, d]
                        o[:, a] = y'
                    end
                end
                Wo = Wo .+ o' * o
            end
        end
        Wo = Wo * (dt / (D * K))
    else
        error("opt must be either :c for controllability gramian, or :o for observability gramian")
    end
end

"""
    min_HSV()

A function for computation of minimum Hankel Singular Value (HSV).
"""
function min_HSV(Wc, Wo)
    min_HSV = Wc*Wo |> LinearAlgebra.eigvals |> minimum |> sqrt
end

"""
    ssp2()

A function for Low-Storage Strong-Stability-Preserving Second-Order Runge-Kutta.

# References
[1] https://gramian.de/
[2] https://github.com/gramian/emgr

# Notes
- STAGES = Configurable number of stages for enhanced stability
- f = function of (x, u, p, t)
- g = function of (x, u, p, t)
"""
function ssp2(f::Function, g::Function, dt, tf, x0, u, p; STAGES=3)
    nt = Int(floor(tf / dt) + 1)
    @assert nt > 0
    y0 = g(x0, u(0), p, 0)
    y = zeros(length(y0), nt)  
    y[:,1] = y0

    xk1 = x0
    xk2 = x0
    for k = 2:nt
        tk = (k - 1.5) * dt
        uk = u(tk)
        for s = 1:(STAGES - 1)
            xk1 = xk1 + (dt / (STAGES - 1)) * f(xk1, uk, p, tk)
        end
        xk2 = xk2 + dt * f(xk1, uk, p, tk)
        xk2 = xk2 / STAGES
        xk2 = xk2 + xk1 * ((STAGES - 1) / STAGES)
        xk1 = xk2
        y[:,k] = g(xk1, uk, p, tk)
    end
    y
end
