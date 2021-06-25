using FaultTolerantControl
const FTC = FaultTolerantControl
using UnPack
using Plots
using Transducers
using LinearAlgebra
using DifferentialEquations
using JLD2, FileIO
using Printf


function run_sim(method, dir_log, file_name="switching.jld2")
    mkpath(dir_log)
    file_path = joinpath(dir_log, file_name)
    saved_data = nothing
    data_exists = isfile(file_path)
    if !data_exists
        _multicopter = LeeHexacopterEnv()
        u_max = (1/3) * _multicopter.m * _multicopter.g * ones(_multicopter.dim_input)
        multicopter = LeeHexacopterEnv(u_max=u_max)
        @unpack m, B, u_min, dim_input = multicopter
        τ = 0.2
        fdi = DelayFDI(τ)
        faults = FaultSet(
                          LoE(3.0, 1, 0.3),  # t, index, level
                          LoE(5.0, 3, 0.1),
                         )  # Note: antisymmetric configuration of faults can cause undesirable control allocation; sometimes it is worse than multiple faults of rotors in symmetric configuration.
        plant = FTC.DelayFDI_Plant(multicopter, fdi, faults)
        @unpack multicopter = plant
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
        p0, x0 = nothing, nothing
        if method == :adaptive || method == :adaptive2static
            p0 = :adaptive
            x0 = State(env_adaptive)()  # start with adaptive CA
        elseif method == :static
            p0 = :static
            x0 = State(env_static)()  # start with static CA
        else
            error("Invalid method")
        end
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
        __log_indicator__ = __LOG_INDICATOR__()
        affect! = (integrator) -> error("Invalid method")
        if method == :adaptive2static
            affect! = (integrator) -> integrator.p = :static
        elseif method == :adaptive || method == :static
            affect! = (integrator) -> nothing
        end
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
        tf = 15.0
        @time prob, df = sim(
                             x0,
                             dynamics!,
                             p0;
                             tf=tf,
                             savestep=0.01,
                             callback=cb,
                            )
        FileIO.save(file_path, Dict("df" => df,
                                    "dim_input" => dim_input,
                                    "u_max" => u_max,
                                    "u_min" => u_min,
                                    "pos_cmd_func" => pos_cmd_func,
                                   ))
    end
    saved_data = JLD2.load(file_path)
end

function plot_figures(dir_log, saved_data)
    @unpack df, dim_input, u_max, u_min, pos_cmd_func = saved_data
    # plots
    ts = df.time
    poss = df.sol |> Map(datum -> datum.plant.state.p) |> collect
    xs = poss |> Map(pos -> pos[1]) |> collect
    ys = poss |> Map(pos -> pos[2]) |> collect
    zs = poss |> Map(pos -> pos[3]) |> collect
    poss_desired = ts |> Map(pos_cmd_func) |> collect
    xs_des = poss_desired |> Map(pos -> pos[1]) |> collect
    ys_des = poss_desired |> Map(pos -> pos[2]) |> collect
    zs_des = poss_desired |> Map(pos -> pos[3]) |> collect
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
    ts_tick = ts[1:100:end]
    tstr = ts_tick |> Map(t -> @sprintf("%0.2f", t)) |> collect
    tstr_empty = ts_tick |> Map(t -> "") |> collect
    ## pos
    p_pos = plot(;
                 title="position",
                 legend=:topleft,
                )
    xticks!(ts_tick, tstr_empty)
    plot!(p_pos, ts, xs;
          label="x",
          color=1,  # i'th default color
         )
    plot!(p_pos, ts, ys;
          label="y",
          color=2,  # i'th default color
         )
    plot!(p_pos, ts, zs;
          label="z",
          color=3,  # i'th default color
         )
    plot!(p_pos, ts, xs_des;
          label="x_des", ls=:dash,
          color=1,  # i'th default color
         )
    plot!(p_pos, ts, ys_des;
          label="y_des", ls=:dash,
          color=2,  # i'th default color
         )
    plot!(p_pos, ts, zs_des;
          label="z_des", ls=:dash,
          color=3,  # i'th default color
         )
    ## Λ
    p__Λ = plot(ts, hcat(_Λ̂s...)'; title="effectiveness vector",
                ylim=(-0.1, 1.1),
                label=["estimated" fill(nothing, dim_input-1)...],
                color=:black,
         )
    xticks!(ts_tick, tstr_empty)
    plot!(p__Λ, ts, hcat(_Λs...)';
                label=["true" fill(nothing, dim_input-1)...],
                color=:red,
                ls=:dash,
                legend=:bottomleft,
               )
    ## method
    p_method = plot(;
                    title="method (adaptive: $(_method_dict[:adaptive]), static: $(_method_dict[:static]))",
                    legend=:topleft,
                   )
    plot!(p_method, ts, hcat(_methods...)';
          label="",
          color=:black,
          ylim=(-0.1, 1.1),
         )
    xticks!(ts_tick, tstr)
    ### states
    p_state = plot(p_pos, p__Λ, p_method;
                   link=:x,  # aligned x axes
                   layout=(3, 1), size=(600, 600),
                  )
    savefig(p_state, joinpath(dir_log, "state.pdf"))
    ## u
    p_u = plot(;
               title="rotor input",
               legend=:topleft,
              )
    xticks!(ts_tick, tstr_empty)
    plot!(p_u, ts, hcat(us_actual...)';
          # ylim=(-0.1*maximum(u_max), 1.1*maximum(u_max)),
          label=["input" fill(nothing, dim_input-1)...],
          color=:black,
          ylim=(minimum(u_min)-1, maximum(u_max)+11)
         )
    plot!(p_u, ts, maximum(u_max)*ones(size(ts));
          label="input min/max",
          ls=:dash,
          color=:red,
         )
    plot!(p_u, ts, minimum(u_min)*ones(size(ts));
          label=nothing,
          ls=:dash,
          color=:red,
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
    ## ν
    p_ν = plot(;
               title="virtual input",
               legend=:left,
              )
    xticks!(ts_tick, tstr_empty)
    plot!(p_ν, ts, hcat(νs...)';
          label=["actual input" fill(nothing, 4-1)...],
          color=:black,
         )
    plot!(p_ν, ts, hcat(νds...)';
          label=["desired input" fill(nothing, 4-1)...],
          color=:red,
          ls=:dash,
         )
    ### inputs
    p_input = plot(p_u, p_ν, p_method;
                   link=:x,  # aligned x axes
                   layout=(3, 1), size=(600, 600),
                  )
    savefig(p_input, joinpath(dir_log, "input.pdf"))
end

function test()
    dir_log = "data"
    methods = [:adaptive, :static, :adaptive2static]
    @show methods
    for method in methods
        @show method
        _dir_log = joinpath(dir_log, String(method))
        saved_data = run_sim(method, _dir_log)
        plot_figures(_dir_log, saved_data)
    end
end
