"""
Common dynamics of MulticopterEnv.
# Variables
f ∈ R: total thrust
M ∈ R^3: moment
"""
function Dynamics!(env::MulticopterEnv;
        faults::Vector{T} where T <: AbstractFault=AbstractFault[],
    )
    actuator_faults = faults |> Filter(fault -> typeof(fault) <: AbstractActuatorFault) |> collect
    actuator_faults_groups = SplitApplyCombine.group(fault -> fault.index, actuator_faults)  # actuator_faults groups classified by fault index
    return function (dX, X, p, t; u)
        u_saturated = FlightSims.saturate(env, u)  # for manual saturation, extend this method
        _u_faulted = u_saturated
        # actuator faults
        for actuator_faults_group in actuator_faults_groups
            last_actuator_fault = select_last_before_t(actuator_faults_group, t)
            _u_faulted = last_actuator_fault(t, _u_faulted)
        end
        u_faulted = _u_faulted
        # @show u_faulted ./ u_saturated, t  # for debugging
        ν = FlightSims.input_to_force_moment(env, u_faulted)  # for manual input_to_force_moment transformation, extend this method
        f, M = ν[1], ν[2:4]
        FlightSims.__Dynamics!(env)(dX, X, p, t; f=f, M=M)
    end
end
