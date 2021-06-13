"""
Dynamical equations with faults for MulticopterEnv.
# Variables
f ∈ R: total thrust
M ∈ R^3: moment
Λ: effectiveness matrix
"""
function Dynamics!(multicopter::MulticopterEnv)
    return function (dX, X, p, t; u, Λ::Diagonal)
        @assert all(diag(Λ) .>= 0.0) && all(diag(Λ) .<= 1.0)
        Λ = Matrix(Λ)  # to assign off-diagonal terms for diffeq
        u_saturated = FlightSims.saturate(multicopter, u)  # for manual saturation, extend this method
        u_faulted = Λ * u_saturated
        # @show u_faulted ./ u_saturated, t  # for debugging
        ν = FlightSims.input_to_force_moment(multicopter, u_faulted)  # for manual input_to_force_moment transformation, extend this method
        f, M = ν[1], ν[2:4]
        FlightSims.__Dynamics!(multicopter)(dX, X, p, t; f=f, M=M)
    end
end
