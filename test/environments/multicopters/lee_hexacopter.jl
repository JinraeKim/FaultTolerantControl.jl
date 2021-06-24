using FaultTolerantControl
using LinearAlgebra


function test()
    env = LeeHexacopterEnv()
    x0 = State(env)()
    prob, df = sim(
                   x0,
                   apply_inputs(Dynamics!(env); u=ones(6), Λ=Diagonal(ones(6)));
                   tf=10.0,
                  )
    df
end
