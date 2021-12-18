using FaultTolerantControl
using Transducers
using Plots


function main()
    N = 4
    d = 2
    θ = [[0, 0], [0.3, 0.1], [0.5, 0.2], [0.7, 0.3], [1, 1]]
    θ_x = θ |> Map(_θ -> _θ[1]) |> collect
    θ_y = θ |> Map(_θ -> _θ[2]) |> collect
    t0 = 0.0
    tf = 1.0
    curve = Bezier(θ, t0, tf)
    curve_params = t0:0.01:tf
    points = curve_params |> Map(curve) |> collect
    points_x = points |> Map(point -> point[1]) |> collect
    points_y = points |> Map(point -> point[2]) |> collect
    fig = plot(points_x, points_y; label="Bezier curve")
    plot!(fig, θ_x, θ_y; st=:scatter, label="control points")
end
