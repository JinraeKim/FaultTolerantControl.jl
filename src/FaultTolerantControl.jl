module FaultTolerantControl

# if you don't reexport `FlightSims`, many APIs, e.g., `State` would report an issue like:
# ERROR: Define the structure of state for your environment
using Reexport
@reexport using FlightSims
using LinearAlgebra
using UnPack

# Fault
export AbstractFault
export AbstractActuatorFault, LoE
# FDI (fault detection and isoltion)
export AbstractFDI, LPFFDI

include("Fault.jl")
include("FDI.jl")


end
