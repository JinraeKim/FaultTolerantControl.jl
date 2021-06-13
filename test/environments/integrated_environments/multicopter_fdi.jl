using FaultTolerantControl
using LinearAlgebra
using UnPack


function test()
    τ = 0.1
    fdi = LPFFDI(τ)
    multicopter = LeeHexacopterEnv()
    x0 = State(multicopter, fdi)()
    @unpack dim_input = multicopter
    Λ_func = function (t)
        _Λ = ones(dim_input)
        if t > 5.0
            _Λ[1] = 0.0
        end
        Λ = _Λ |> Diagonal
    end
    tf = 10.0
    prob, sol = sim(x0, apply_inputs(Dynamics!(multicopter, fdi);
                                     u=zeros(dim_input), Λ=ones(dim_input) |> Diagonal,);
                    tf=tf)
end
