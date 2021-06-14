abstract type AbstractFault end
abstract type AbstractActuatorFault <: AbstractFault end

# API
affect!(fault::AbstractActuatorFault, Λ, t) = error("Not implemented fault")

# FaultSet
FaultSet(args...) = AbstractFault[args...]

function Affect!(faults::Vector{AbstractActuatorFault})
    return function (Λ, t)
        actuator_faults_groups = SplitApplyCombine.group(fault -> fault.index, faults)  # actuator_faults groups classified by fault index
        for actuator_faults_group in actuator_faults_groups
            last_actuator_fault = select_last_before_t(actuator_faults_group, t)
            affect!(last_actuator_fault, Λ, t)
        end
    end
end

# select_last
"""
Select the last fault (concerning the applied time) among given faults.
"""
function select_last_before_t(faults::Vector{T} where T <: AbstractFault, t)
    fault_times_less_than_t = faults |> Map(fault -> fault.time) |> Filter(<=(t)) |> collect
    if length(fault_times_less_than_t) == 0
        # case 1: if there is no faults occured before t
        return (t, u) -> u
    else
        # case 2: if there are some faults occured before t
        fault_last_indices = findall(fault -> fault.time == maximum(fault_times_less_than_t), faults)
        fault_last_idx = length(fault_last_indices) == 1 ? fault_last_indices[1] : error("Among given faults, more than one faults occured at the same time")
        return faults[fault_last_idx]
    end
end


"""
Loss of effectiveness (LoE).
"""
struct LoE <: AbstractActuatorFault
    time::Real
    index
    level::Real
    function LoE(time, index, level)
        @assert level >= 0.0 && level <= 1.0
        new(time, index, level)
    end
end

"""
# Variables
Λ ∈ R^(n×n): effectiveness matrix
_Λ ∈ R^n: effectiveness vector
"""
function affect!(fault::LoE, Λ, t)
    @unpack time, index, level = fault
    _Λ = ones(size(u))
    if t >= time
        _Λ[index] = level
    end
    Λ .= Λ * Diagonal(_Λ) |> Matrix
end
