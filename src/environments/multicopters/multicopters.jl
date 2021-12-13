"""
Dynamical equations with faults for Multicopter.
# Variables
f ∈ R: total thrust
M ∈ R^3: moment
Λ: effectiveness matrix
"""
function Dynamics!(multicopter::Multicopter)
    @Loggable function dynamics!(dx, x, p, t; u, Λ)
        @assert all(diag(Λ) .>= 0.0) && all(diag(Λ) .<= 1.0)
        @nested_log :FDI Λ = Matrix(Λ)  # to assign off-diagonal terms for diffeq (if Λ <: Diagonal)
        @nested_onlylog :input u_cmd = u
        @nested_log :input u_saturated = FSimZoo.saturate(multicopter, u)
        @nested_log :input u_faulted = Λ * u_saturated
        @nested_onlylog :input u_actual = u_faulted
        # @show u_faulted ./ u_saturated, t  # for debugging
        @nested_log :input ν = FSimZoo.input_to_force_moment(multicopter, u_faulted)  # for manual input_to_force_moment transformation, extend this method
        f, M = ν[1], ν[2:4]
        @nested_log FSimZoo.__Dynamics!(multicopter)(dx, x, p, t; f=f, M=M)  # :state, :input
    end
end
