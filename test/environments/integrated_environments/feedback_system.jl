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
                      LoE(5.0, 1, 0.0),
                      LoE(5.0, 2, 0.0),
                     )  # Note: antisymmetric configuration of faults can cause undesirable control allocation; sometimes it is worse than multiple faults of rotors in symmetric configuration.
    plant = FTC.DelayFDI_Plant(multicopter, fdi, faults)
    @unpack multicopter = plant
    @unpack m, B, u_max, u_min, dim_input = multicopter
    pos_cmd_func = (t) -> [2, 1, 3]
    controller = BacksteppingPositionControllerEnv(m; pos_cmd_func=pos_cmd_func)
    allocator = PseudoInverseAllocator(B)
    # allocator = ConstrainedAllocator(B, u_min, u_max)
    control_system = FTC.BacksteppingControl_StaticAllocator_ControlSystem(controller, allocator)
    env = FTC.DelayFDI_Plant_BacksteppingControl_PseudoInverseCA_FeedbackSystem(
                                                                                plant,
                                                                                control_system,
                                                                               )
    # sim
    tf = 20.0
    x0 = State(env)()
    @time prob, df = sim(
                   x0,
                   Dynamics!(env);
                   tf=tf,
                   savestep=0.01,
                  )
    ts = df.time
    poss = df.plant |> Map(datum -> datum.state.p) |> collect
    us_cmd = df.plant |> Map(datum -> datum.input.u_cmd) |> collect
    us_actual = df.plant |> Map(datum -> datum.input.u_actual) |> collect
    us_saturated = df.plant |> Map(datum -> datum.input.u_saturated) |> collect
    us_cmd_faulted = df.plant |> Map(datum -> datum.FDI.Λ * datum.input.u_cmd) |> collect
    νs = df.plant |> Map(datum -> datum.input.ν) |> collect
    νds = df.νd
    Λs = df.plant |> Map(datum -> datum.FDI.Λ) |> collect
    Λ̂s = df.plant |> Map(datum -> datum.FDI.Λ̂) |> collect
    _Λs = Λs |> Map(diag) |> collect
    _Λ̂s = Λ̂s |> Map(diag) |> collect
    # plots
    p_pos = plot(ts, hcat(poss...)';
                 title="position",
                 label=["x" "y" "z"],
                 legend=:topright,
                )
    p__Λ = plot(ts, hcat(_Λs...)'; title="effectiveness matrix",
                label=["true" fill(nothing, dim_input-1)...],
                color="black",
                ls=:dash,
                legend=:topright,
               )
    plot!(p__Λ, ts, hcat(_Λ̂s...)';
          label=["estimated" fill(nothing, dim_input-1)...],
          color="red")
    p_u = plot(;
               title="rotor input",
               legend=:topright,
              )
    plot!(p_u, ts, maximum(u_max)*ones(size(ts));
          label="input min/max",
          ls=:dash,
          color=:blue,
         )
    plot!(p_u, ts, minimum(u_min)*ones(size(ts));
          label=nothing,
          ls=:dash,
          color=:blue,
         )
    plot!(p_u, ts, hcat(us_actual...)';
          # ylim=(-0.1*maximum(u_max), 1.1*maximum(u_max)),
          label=["input" fill(nothing, dim_input-1)...],
          color="red",
         )
    # plot!(p_u, ts, hcat(us_cmd...)';
    #       label=["command" fill(nothing, dim_input-1)...],
    #       color="black",
    #       ls=:dash,
    #      )
    # plot!(p_u, ts, hcat(us_saturated...)';
    #       # ylim=(-0.1*maximum(u_max), 1.1*maximum(u_max)),
    #       label=["saturated command" fill(nothing, dim_input-1)...],
    #       color="green",
    #      )
    plot!(p_u, ts, hcat(us_cmd_faulted...)';
          label=["faulted command" fill(nothing, dim_input-1)...],
          color="orange",
          ls=:dash,
         )
    p_ν = plot(;
               title="virtual input",
               legend=:topright,
              )
    plot!(p_ν, ts, hcat(νs...)';
          label=["actual input" fill(nothing, 4-1)...],
          color=:red,
         )
    plot!(p_ν, ts, hcat(νds...)';
          label=["desired input" fill(nothing, 4-1)...],
          color=:black,
          ls=:dash,
         )
    plot(p_pos, p__Λ, p_u, p_ν; layout=(4, 1), size=(600, 600))
end
