"""
Bezier curve
θ: a vector whose length is (N+1); each element can also be vector,
e.g., θ = [[1, 2, 3], [2, 3, 4]]
"""
function Bezier(θ::AbstractVector; t0::Number=0.0, tf::Number=1.0)
    N = length(θ)-1
    function bezier(t)
        s = (t-t0)/(tf-t0)
        basis = 0:N |> Map(i -> binomial(N, i) * s^i * (1-s)^(N-i)) |> collect
        basis .* θ |> sum  # sum of N_C_i * s^(N-i) * (1-s)^i * θ_i
    end
end
