using FaultTolerantControl
const FTC = FaultTolerantControl
using UnPack


function test()
    multicopter = LeeHexacopterEnv()
    @unpack m, g, B = multicopter
    m = copy(m)
    g = copy(g)
    pos0 = zeros(3)
    controller = BacksteppingPositionControllerEnv(copy(m))
    allocator = PseudoInverseAllocator(copy(B))
    cs = FTC.BacksteppingConrtol_PseudoInverseCA_ControlSystem(controller, allocator)
    x0 = State(cs)(pos0, m, g)
    # simulation
    tf = 10.0
    prob, df = sim(x0,
                   apply_inputs(Dynamics!(cs); pos_cmd=[2, 1, 3], Ṫd=0.0);
                   tf=tf,
                  )
    df
end
