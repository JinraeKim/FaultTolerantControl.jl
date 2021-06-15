module FaultTolerantControl

# if you don't reexport `FlightSims`, many APIs, e.g., `State` would report an issue like:
# ERROR: Define the structure of state for your environment
using Reexport
@reexport using FlightSims
const FS = FlightSims
import FlightSims: State, Params, Dynamics!, Dynamics, DatumFormat, Process  # `import` will help you to automatically extend the imported methods
using DifferentialEquations
using LinearAlgebra
using Transducers, UnPack, ComponentArrays
import SplitApplyCombine
import MatrixEquations

# Fault
export AbstractFault, FaultSet
export AbstractActuatorFault, LoE
# FDI (fault detection and isoltion)
export AbstractFDI, LPFFDI, DelayFDI
# allocator
export PseudoInverseControlAllocator
# controllers
export BacksteppingPositionControllerEnv

include("faults.jl")
include("environments/environments.jl")


end
