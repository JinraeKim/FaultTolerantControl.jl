using FaultTolerantControl
using LinearAlgebra
using Transducers


function test()
    multicopter = IslamQuadcopterEnv()
    x0 = State(multicopter)()
    Λ_func = function (t)
        _Λ = ones(4)
        if t >= 0.1
            _Λ[1] = 0.1
        end
        if t >= 0.2
            _Λ[2] = 0.5
        end
        _Λ |> Diagonal |> Matrix
    end
    Λ_func_compat = (x, p, t) -> Λ_func(t)
    tf = 10.0
    Δt = 0.01
    prob, sol, df = sim(
                        x0,
                        apply_inputs(Dynamics!(multicopter); u=ones(4), Λ=Λ_func_compat);
                        datum_format=save_inputs(DatumFormat(multicopter);
                                                 Λ=Λ_func_compat),
                        saveat=0.0:Δt:tf,
                        tf=tf,
                       )
    ts = df.time
    _Λs = df.Λ |> Map(diag) |> collect
    _Λs
end
