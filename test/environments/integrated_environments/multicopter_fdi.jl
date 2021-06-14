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
    index = 1
    faults = FaultSet(LoE(5.0, index, 0.5))
    envs = (multicopter, fdi, faults)
    x0 = State(envs...)()
    tf = 10.0
    prob, sol, df = sim(x0,
                        apply_inputs(Dynamics!(envs...);
                                     u=zeros(6),
                                    );
                        datum_format=DatumFormat(envs...),
                        savestep=0.01,
                        tf=tf)
    ts = df.time
    _Λs_index = df.Λ |> Map(Λ -> diag(Λ)[index]) |> collect
    _Λ̂s_index = df.Λ̂ |> Map(Λ̂ -> diag(Λ̂)[index]) |> collect
    # plot
    p = plot()
    plot!(ts, hcat(_Λs_index...)'; label="true")
    plot!(ts, hcat(_Λ̂s_index...)'; label="estimated")
end
