using FaultTolerantControl
using Transducers
using Plots


function test()
    m = 1.0
    pos0 = zeros(3)
    g = 9.81
    controller = BacksteppingPositionControllerEnv(m)
    x0_controller = State(controller)(pos0, m, g)
    prob, sol = sim(x0_controller,
                    apply_inputs(Dynamics!(controller),
                                 pos_cmd=[2, 1, 3], Ṫd=1.0);
                    tf=10.0)
    df = Process(controller)(prob, sol; Δt=0.01)
    ts = df.time
    xds = df.state |> Map(state -> state.ref_model.x_0) |> collect
    plot(ts, hcat(xds...)')
    # Tds = df.state |> Map(state -> state.Td) |> collect
    # plot(ts, hcat(Tds...)')
end
