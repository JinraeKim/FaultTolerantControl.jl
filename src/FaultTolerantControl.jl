module FaultTolerantControl

# if you don't reexport `FlightSims`, many APIs, e.g., `State` would report an issue like:
# ERROR: Define the structure of state for your environment
# using Debugger  # tmp
using Reexport
@reexport using FlightSims
const FS = FlightSims
# `import` will help you to automatically extend the imported methods
import FlightSims: State, Params, Dynamics!, Dynamics
import FlightSims: Command
using LinearAlgebra
using Convex, Mosek, MosekTools
using Transducers, UnPack, ComponentArrays
import SplitApplyCombine
import MatrixEquations
using NumericalIntegration
using ControlSystems: ss, gram

# Fault
export AbstractFault, FaultSet
export AbstractActuatorFault, LoE
# FDI (fault detection and isoltion)
export AbstractFDI, LPFFDI, DelayFDI
# allocator
export PseudoInverseAllocator, ConstrainedAllocator, AdaptiveAllocator
# trajectory generation
export Bezier
# cost functional
export PositionAngularVelocityCostFunctional
export cost
# reconfigurability
export ssom
export empirical_gramian


include("utils.jl")
include("costs.jl")
include("reconfigurability.jl")
include("faults.jl")
include("environments/environments.jl")
include("trajectory_generation.jl")
include("reconfigurability.jl")


end
