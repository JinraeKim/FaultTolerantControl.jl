using FaultTolerantControl
const FTC = FaultTolerantControl


function test()
    allocator = AdaptiveAllocator()
    x0 = State(allocator)()
    tf = 10.0
    prob, df = sim(
                   x0,
                   apply_inputs(Dynamics!(allocator); Θ̂̇=zeros(6, 4));
                   tf=tf
                  )
    df
end
