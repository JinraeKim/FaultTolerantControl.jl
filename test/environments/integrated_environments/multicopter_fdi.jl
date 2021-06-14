using FaultTolerantControl
using LinearAlgebra
using UnPack
using Transducers
using Plots


function test_LPFFDI()
    τ = 0.1
    fdi = LPFFDI(τ)
    multicopter = LeeHexacopterEnv()
    x0 = State(multicopter, fdi)()
    @unpack dim_input = multicopter
    Λ_func = function (t)
        _Λ = ones(dim_input)
        if t > 5.0
            _Λ[1] = 0.0
        end
        Λ = _Λ |> Diagonal
    end
    tf = 10.0
    prob, sol = sim(x0, apply_inputs(Dynamics!(multicopter, fdi);
                                     u=zeros(dim_input), Λ=ones(dim_input) |> Diagonal,);
                    tf=tf)
end

function test()
    multicopter = LeeHexacopterEnv()
    τ = 0.2
    fdi = DelayFDI(τ)
    faults = FaultSet(LoE(5.0, 1, 0.5))
    envs = (multicopter, fdi, faults)
    x0 = State(envs...)()
    tf = 10.0
    prob, sol, df = sim(x0,
                        apply_inputs(Dynamics!(envs...);
                                     u=zeros(6),
                                    );
                        datum_format=DatumFormat(envs...),
                        tstep=0.01,
                        tf=tf)
    ts = df.time
    _Λs = df.Λ |> Map(diag) |> collect
    _Λ̂s = df.Λ̂ |> Map(diag) |> collect
    # plot
    p = plot()
    plot!(ts, hcat(_Λs...)')
    plot!(ts, hcat(_Λ̂s...)')
end
