abstract type AbstractAllocator end
abstract type StaticAllocator <: AbstractAllocator end
include("PseudoInverseAllocator.jl")
include("ConstrainedAllocator.jl")
include("AdaptiveAllocator.jl")
