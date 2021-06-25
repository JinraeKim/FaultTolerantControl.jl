using FaultTolerantControl
const FTC = FaultTolerantControl
using UnPack
using Plots
using Transducers
using LinearAlgebra
using DifferentialEquations
using JLD2, FileIO


function run_sim(dir_log, file_name="switching.jld2")
    mkpath(dir_log)
    file_path = joinpath(dir_log, file_name)
    saved_data = nothing
    data_exists = isfile(file_path)
    if data_exists
        saved_data = JLD2.load(file_path)
    else
        _multicopter = LeeHexacopterEnv()
        u_max = _multicopter.u_max ./ 3
        multicopter = LeeHexacopterEnv(u_max=u_max)
        τ = 0.2
        fdi = DelayFDI(τ)
        faults = FaultSet(
                          LoE(5.0, 1, 0.5),  # t, index, level
                          LoE(5.0, 3, 0.0),
                         )  # Note: antisymmetric configuration of faults can cause undesirable control allocation; sometimes it is worse than multiple faults of rotors in symmetric configuration.
        plant = FTC.DelayFDI_Plant(multicopter, fdi, faults)
        @unpack multicopter = plant
        @unpack m, B, u_min, dim_input = multicopter
        pos_cmd_func = (t) -> [2, 1, 3]
        controller = BacksteppingPositionControllerEnv(m; pos_cmd_func=pos_cmd_func)
        # static allocators
        # allocator = PseudoInverseAllocator(B)  # deprecated; it does not work when failures occur. I guess it's due to Moore-Penrose pseudo inverse.
        allocator = ConstrainedAllocator(B, u_min, u_max)
        control_system_static = FTC.BacksteppingControl_StaticAllocator_ControlSystem(controller, allocator)
        env_static = FTC.DelayFDI_Plant_BacksteppingControl_StaticAllocator_ControlSystem(plant, control_system_static)
        # adaptive allocators
        allocator = AdaptiveAllocator(B)
        control_system_adaptive = FTC.BacksteppingControl_AdaptiveAllocator_ControlSystem(controller, allocator)
        env_adaptive = FTC.DelayFDI_Plant_BacksteppingControl_AdaptiveAllocator_ControlSystem(plant, control_system_adaptive)
        p0 = :adaptive
        x0 = State(env_adaptive)()  # start with adaptive CA
        # p0 = :static
        # x0 = State(env_static)()  # start with static CA
        @Loggable function dynamics!(dx, x, p, t)
            @log method = p
            if p == :adaptive
                @nested_log Dynamics!(env_adaptive)(dx, x, p, t)
            elseif p == :static
                @nested_log Dynamics!(env_static)(dx, x, p, t)
            else
                error("Invalid method")
            end
        end
        # callback; TODO: a fancy way of utilising callbacks like SimulationLogger.jl...?
        affect!(integrator) = integrator.p = :static
        __log_indicator__ = __LOG_INDICATOR__()
        condition = function (x, t, integrator)
            p = integrator.p
            if p == :adaptive
                x = copy(x)
                dict = Dynamics!(env_adaptive)(zero.(x), x, p, t, __log_indicator__)
                u_actual = dict[:plant][:input][:u_actual]
                is_switched = any(u_actual .>= u_max) || any(u_actual .<= u_min)
                return is_switched
            elseif p == :static
                return false
            else
                error("Invalid method")
            end
        end
        cb_switch = DiscreteCallback(condition, affect!)
        cb = CallbackSet(cb_switch)
        # sim
        tf = 10.0
        @time prob, df = sim(
                             x0,
                             dynamics!, p0;
                             tf=tf,
                             savestep=0.01,
                             callback=cb,
                            )
        FileIO.save(file_path, Dict("df" => df, "dim_input" => dim_input, "u_max" => u_max, "u_min" => u_min))
    end
    saved_data
end

function test()
    dir_log = "data"
    saved_data = run_sim(dir_log)
    @unpack df, dim_input, u_max, u_min = saved_data
    # plots
    ts = df.time
    poss = df.sol |> Map(datum -> datum.plant.state.p) |> collect
    us_cmd = df.sol |> Map(datum -> datum.plant.input.u_cmd) |> collect
    us_actual = df.sol |> Map(datum -> datum.plant.input.u_actual) |> collect
    us_saturated = df.sol |> Map(datum -> datum.plant.input.u_saturated) |> collect
    us_cmd_faulted = df.sol |> Map(datum -> datum.plant.FDI.Λ * datum.plant.input.u_cmd) |> collect
    νs = df.sol |> Map(datum -> datum.plant.input.ν) |> collect
    νds = df.sol |> Map(datum -> datum.νd) |> collect
    Λs = df.sol |> Map(datum -> datum.plant.FDI.Λ) |> collect
    Λ̂s = df.sol |> Map(datum -> datum.plant.FDI.Λ̂) |> collect
    _Λs = Λs |> Map(diag) |> collect
    _Λ̂s = Λ̂s |> Map(diag) |> collect
    _method_dict = Dict(:adaptive => 0, :static => 1)
    _methods = df.sol |> Map(datum -> _method = datum.method == :adaptive ? _method_dict[:adaptive] : _method_dict[:static]) |> collect
    # plots
    ## states
    ### pos
    p_pos = plot(ts, hcat(poss...)';
                 title="position",
                 label=["x" "y" "z"],
                 legend=:topleft,
                )
    ### Λ
    p__Λ = plot(ts, hcat(_Λs...)'; title="effectiveness vector",
                label=["true" fill(nothing, dim_input-1)...],
                color=:red,
                ls=:dash,
                legend=:bottomleft,
               )
    plot!(p__Λ, ts, hcat(_Λ̂s...)';
          label=["estimated" fill(nothing, dim_input-1)...],
          color=:black,
         )
    p_state = plot(p_pos, p__Λ;
                   link=:x,  # aligned x axes
                   layout=(2, 1),
                  )
    savefig(p_state, joinpath(dir_log, "state.pdf"))
    ## inputs
    ### u
    p_u = plot(;
               title="rotor input",
               legend=:topleft,
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
          color=:black,
          ylim=(minimum(u_min)-1, maximum(u_max)+11)
         )
    # plot!(p_u, ts, hcat(us_cmd...)';
    #       label=["command" fill(nothing, dim_input-1)...],
    #       color=:red,
    #       ls=:dash,
    #      )
    # plot!(p_u, ts, hcat(us_saturated...)';
    #       # ylim=(-0.1*maximum(u_max), 1.1*maximum(u_max)),
    #       label=["saturated command" fill(nothing, dim_input-1)...],
    #       color="green",
    #      )
    # plot!(p_u, ts, hcat(us_cmd_faulted...)';
    #       label=["faulted command" fill(nothing, dim_input-1)...],
    #       color="orange",
    #       ls=:dash,
    #      )
    ### ν
    p_ν = plot(;
               title="virtual input",
               legend=:left,
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
    ### method
    p_method = plot(;
                    title="method (adaptive: $(_method_dict[:adaptive]), static: $(_method_dict[:static]))",
                    legend=:topleft,
                   )
    plot!(p_method, ts, hcat(_methods...)';
          label="",
          color=:black,
          ylim=(-0.1, 1.1),
         )
    p_input = plot(p_u, p_ν, p_method;
                   link=:x,  # aligned x axes
                   layout=(3, 1), size=(600, 600),
                  )
    savefig(p_input, joinpath(dir_log, "input.pdf"))
end
