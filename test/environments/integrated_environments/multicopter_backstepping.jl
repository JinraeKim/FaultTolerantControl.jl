using FaultTolerantControl
const FTC = FaultTolerantControl
using UnPack
using Plots
using Transducers
using LinearAlgebra


function test()
    multicopter = LeeHexacopterEnv()
    τ = 0.2
    fdi = DelayFDI(τ)
    faults = FaultSet(LoE(10.0, 1, 0.2), LoE(10.0, 4, 0.2))  # Note: antisymmetric configuration of faults can cause undesirable control allocation; sometimes it is worse than multiple faults of rotors in symmetric configuration.
    multicopter_fdi_faults = FTC.Multicopter_DelayFDI_Faults(multicopter, fdi, faults)
    @unpack multicopter = multicopter_fdi_faults
    @unpack m, B, u_max, dim_input = multicopter
    pos_cmd_func = (t) -> [2, 1, 3]
    controller = BacksteppingPositionControllerEnv(m; pos_cmd_func=pos_cmd_func)
    allocator = PseudoInverseControlAllocator(B)
    env = FTC.Multicopter_DelayFDI_Faults_BacksteppingControl_PseudoInverseCA(
                                                                              multicopter_fdi_faults,
                                                                              controller,
                                                                              allocator,
                                                                             )
    # sim
    tf = 40.0
    x0 = State(env)()
    prob, sol, df = sim(
                        x0,
                        Dynamics!(env);
                        datum_format=DatumFormat(env),
                        tf=tf,
                       )
    ts = df.time
    poss = df.pos
    us_cmd = df.u_cmd
    Λs = df.Λ
    Λ̂s = df.Λ̂
    _Λs = Λs |> Map(diag) |> collect
    _Λ̂s = Λ̂s |> Map(diag) |> collect
    # plots
    p_pos = plot(ts, hcat(poss...)')
    p__Λ = plot(ts, hcat(_Λs...)'; title="effectiveness matrix",
                label=["true" fill(nothing, dim_input-1)...],
                color="black")
    plot!(p__Λ, ts, hcat(_Λ̂s...)';
          label=["estimated" fill(nothing, dim_input-1)...],
          color="red")
    p_u_cmd = plot(ts, hcat(us_cmd...)'; title="rotor input", label=nothing, ylim=(0, maximum(u_max)))
    plot(p_pos, p__Λ, p_u_cmd; layout=(3, 1), size=(600, 600))
end
