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
    faults = FaultSet(
                      LoE(10.0, 1, 0.2),
                      LoE(10.0, 4, 0.2),
                     )  # Note: antisymmetric configuration of faults can cause undesirable control allocation; sometimes it is worse than multiple faults of rotors in symmetric configuration.
    plant = FTC.DelayFDI_Plant(multicopter, fdi, faults)
    @unpack multicopter = plant
    @unpack m, B, u_max, u_min, dim_input = multicopter
    pos_cmd_func = (t) -> [2, 1, 3]
    controller = BacksteppingPositionControllerEnv(m; pos_cmd_func=pos_cmd_func)
    allocator = PseudoInverseAllocator(B)
    control_system = FTC.BacksteppingConrtol_PseudoInverseCA_ControlSystem(controller, allocator)
    env = FTC.DelayFDI_Plant_BacksteppingControl_PseudoInverseCA_FeedbackSystem(
                                                                                plant,
                                                                                control_system,
                                                                               )
    # sim
    tf = 40.0
    x0 = State(env)()
    prob, df = sim(
                   x0,
                   Dynamics!(env);
                   tf=tf,
                  )
    ts = df.time
    poss = df.plant |> Map(datum -> datum.state.p) |> collect
    us_cmd = df.plant |> Map(datum -> datum.input.u_cmd) |> collect
    us_actual = df.plant |> Map(datum -> datum.input.u_actual) |> collect
    Λs = df.plant |> Map(datum -> datum.FDI.Λ) |> collect
    Λ̂s = df.plant |> Map(datum -> datum.FDI.Λ̂) |> collect
    _Λs = Λs |> Map(diag) |> collect
    _Λ̂s = Λ̂s |> Map(diag) |> collect
    # plots
    p_pos = plot(ts, hcat(poss...)';
                 title="position",
                 label=["x" "y" "z"],
                 legend=:outertopright,
                )
    p__Λ = plot(ts, hcat(_Λs...)'; title="effectiveness matrix",
                label=["true" fill(nothing, dim_input-1)...],
                color="black",
                ls=:dash,
                legend=:outertopright,
               )
    plot!(p__Λ, ts, hcat(_Λ̂s...)';
          label=["estimated" fill(nothing, dim_input-1)...],
          color="red")
    p_u = plot(;
               title="rotor input",
               legend=:outertopright,
              )
    plot!(p_u, ts, maximum(u_max)*ones(size(ts));
          label="input maximum",
          ls=:dash,
          color=:blue,
         )
    plot!(p_u, ts, minimum(u_min)*ones(size(ts));
          label="input minimum",
          ls=:dash,
          color=:blue,
         )
    plot!(p_u, ts, hcat(us_actual...)';
          # ylim=(-0.1*maximum(u_max), 1.1*maximum(u_max)),
          label=["input" fill(nothing, dim_input-1)...],
          color="red",
         )
    plot!(p_u, ts, hcat(us_cmd...)';
          label=["command" fill(nothing, dim_input-1)...],
          color="black",
          ls=:dash,
         )
    plot(p_pos, p__Λ, p_u; layout=(3, 1), size=(600, 600))
end
