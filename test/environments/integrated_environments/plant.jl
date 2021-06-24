using FaultTolerantControl
const FTC = FaultTolerantControl
using LinearAlgebra
using Transducers
using Plots


function test()
    multicopter = LeeHexacopterEnv()
    τ = 0.2
    fdi = DelayFDI(τ)
    index = 1
    faults = FaultSet(LoE(5.0, index, 0.5))
    env = FTC.DelayFDI_Plant(multicopter, fdi, faults)
    x0 = State(env)()
    tf = 10.0
    prob, df = sim(x0,
                   apply_inputs(Dynamics!(env); u=zeros(6));
                   savestep=0.01,
                   tf=tf)
    ts = df.time
    _Λs_index = df.FDI |> Map(datum -> diag(datum.Λ)[index]) |> collect
    _Λ̂s_index = df.FDI |> Map(datum -> diag(datum.Λ̂)[index]) |> collect
    # plot
    p = plot()
    plot!(ts, hcat(_Λs_index...)'; label="true")
    plot!(ts, hcat(_Λ̂s_index...)'; label="estimated")
end
