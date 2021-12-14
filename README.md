# FaultTolerantControl
[FaultTolerantControl.jl](https://github.com/JinraeKim/FaultTolerantControl.jl) is a package for fault tolerant control (FTC).

## Notes
- This package is based on [FlightSims.jl](https://github.com/JinraeKim/FlightSims.jl).

## Changes in FaultTolerantControl@0.3.0
It is rewritten to be compatible with FlightSims.jl@1.0.0, Julia@1.7.0.

## FTC using various CA (control allocation) methods
See `./test/CA_backstepping.jl`.
Run this at `./`.

![ex_screenshot](./data/adaptive/state.png)
![ex_screenshot](./data/adaptive/input.png)

- Parallel trajectory generation is available. Benchmarks with
1) M1 Macbook Air: 7 threads, about 4s / 7 trajectories
2) Ryzen 5900X: 24 threads, about 5s / 24 trajectories.
Note that the benchmark results may be outdated.

## To-do
- [x] Make a trajectory generator; to save trajectory command
via JLD2 safely (without reconstruction).
- [ ] Complete the main script for the second-year report; see [the related project](https://github.com/JinraeKim/FaultTolerantControl.jl/projects/4).
- [ ] Calculate objective functional

### Notes
- Previously written examples are deprecated; see previous versions, e.g., FaultTolerantControl@0.2.0.

### Funding
This research was supported by Unmanned Vehicles Core Technology Research and Development Program through the National Research Foundation of Korea (NRF) and Unmanned Vehicle Advanced Research Center (UVARC) funded by the Ministry of Science and ICT, the Republic of Korea (2020M3C1C1A01083162).
