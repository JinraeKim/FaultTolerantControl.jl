using FaultTolerantControl
using LinearAlgebra
using Transducers
using Plots


function test_LPFFDI()
    τ = 0.2
    fdi = LPFFDI(τ)
    m = 6
    Λ̂0 = State(fdi)(m)  # input dimension -> Diagonal(ones(m))
    Λ = function (x, p, t)
        Λ = Diagonal(ones(m))
        if t < 5
            Λ = Diagonal(0.5*ones(m))
        elseif t < 10
            Λ = Diagonal(0.1*ones(m))
        else
            Λ = Diagonal(0.0*ones(m))
        end
        Λ
    end
    prob, sol = sim(Λ̂0, apply_inputs(Dynamics!(fdi); Λ=Λ); tf=10.0)
    df = Process(fdi)(prob, sol)
    Λ̂s_diag = df.state |> Map(diag) |> collect
    plot(df.time, hcat(Λ̂s_diag...)')
end

"""
DOES NOT WORK
"""
function test_DelayFDI()
    τ = 0.2
    fdi = DelayFDI(τ)
    dim_input = 6
    x0 = State(fdi)(dim_input)
    Λ_func = function (t)
        Λ = Diagonal(ones(dim_input))
        if t < 5
            Λ = Diagonal(1.0*ones(dim_input))
        elseif t < 10
            Λ = Diagonal(0.5*ones(dim_input))
        else
            Λ = Diagonal(0.1*ones(dim_input))
        end
    end
    prob, sol = sim(x0, Dynamics!(fdi, Λ_func); tf=20.0)
    ts = 0.0:0.01:20.0
    _Λs = ts |> Map(Λ_func) |> Map(diag) |> collect
    _Λ̂s = ts |> Map(t -> sol(t).Λ̂) |> Map(diag) |> collect
    p = plot()
    plot!(ts, hcat(_Λs...)'; color="black")
    plot!(ts, hcat(_Λ̂s...)'; color="blue")
end
