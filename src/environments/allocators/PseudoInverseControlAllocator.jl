"""
Moore-Penrose inverse control allocator.
"""
struct PseudoInverseControlAllocator <: AbstractControlAllocator
    B_pinv
    function PseudoInverseControlAllocator(B)
        B_pinv = pinv(B)
        new(B_pinv)
    end
end

"""
# Variables
ν: virtual input
# Notes
ν = B*u where u: control input
"""
function (allocator::PseudoInverseControlAllocator)(ν, Λ=Diagonal(ones(size(ν))))
    (pinv(Λ) * allocator.B_pinv) * ν  # pinv(B*Λ) = pinv(Λ) * pinv(B)
end
