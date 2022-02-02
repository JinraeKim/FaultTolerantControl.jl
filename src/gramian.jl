"""
	empirical_ctrl_gramian()

A function for computation of controllability gramian.

# Notes
- pr = parameter vector(s), each column i sone parameter sample
- xs = steady-state and nominal initial state x_0
- us = steady-state input
"""
function empirical_ctrl_gramian(f, s; dt::Real, tf::Real, pr::Matrix, xs::Vector)
	M, N, L = s
	P, K = size(pr)
	ut = t -> (t <= dt) / dt
	G(x, u, p, t) = x

	um = ones(M, 1) * Matrix([-1 1])  # input scales

	C = size(um)[2]  # Number of input scales sets
	
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
end

"""
	empirical_obsv_gramian()

A function for computation of observability gramian.

# Notes
- pr = parameter vector(s), each column i sone parameter sample
- xs = steady-state and nominal initial state x_0
- us = steady-state input
"""
function empirical_obsv_gramian(f, g, s; dt::Real, tf::Real, pr::Matrix, xs::Vector, us::Vector)
	M, N, L = s
	P, K = size(pr)
	up = t -> us
	R = L

	xm = ones(N, 1) * Matrix([-1 1])  # initial-state scales

	A = size(xm)[1]  # Number of total states
	D = size(xm)[2]  # Number of state scales sets

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
end

"""
	ssp2()

A function for Low-Storage Strong-Stability-Preserving Second-Order Runge-Kutta.

# Notes
- STAGES = Configurable number of stages for enhanced stability
"""
function ssp2(f, g, dt, tf, x0, u, p; STAGES=3)
    nt = Int(floor(tf / dt) + 1)
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