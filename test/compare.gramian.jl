using FaultTolerantControl
using ControlSystems: ss, gram
using LinearAlgebra


function compare_gramian(opt::Symbol=:gram)
	A = -Matrix(I, 4, 4)
	B = [0.0 1.0 0.0 1.0]'
	C = [0.0 0.0 1.0 1.0]
    D = [0]

	f(x, u, p, t) = A*x + B*u .+ p
	g(x, u, p, t) = C*x

	# System dimension
	n = 4
	m = 1
	l = 1

	x0 = [0.0, 0.0, 0.0, 0.0]
	u0 = [0.0]

    if opt === :empirical
        Wc = empirical_gramian(f, g, m, n, l; opt=:c, dt=0.01, tf=1.0, pr=zeros(4, 1), xs=x0, us=u0)
        Wo = empirical_gramian(f, g, m, n, l; opt=:o, dt=0.01, tf=1.0, pr=zeros(4, 1), xs=x0, us=u0)
    else
        sys = ss(A, B, C, D)

        Wc = gram(sys, :c)
        Wo = gram(sys, :o)
    end
    @show Wc Wo
    nothing
end
