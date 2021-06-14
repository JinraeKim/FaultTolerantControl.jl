# TODO!!!
# abstract type Multicopter_FDI_Faults_Controller <: AbstractEnv end

# struct Multicopter_DelayFDI_Faults_BacksteppingFTC <: Multicopter_FDI_Faults_Controller
# end

# """
# MulticopterEnv + BacksteppingPositionControllerEnv
# """
# function State(
#         multicopter::MulticopterEnv,
#         fdi::DelayFDI,
#         faults::Vector{AbstractFault},
#         controller::BacksteppingPositionControllerEnv,
#     )
#     @unpack m, g = multicopter
#     return function (; args_multicopter=())
#         x_multicopter = State(multicopter)(args_multicopter...)
#         pos0 = copy(x_multicopter.p)
#         x_controller = State(controller)(pos0, m, g)
#         ComponentArray(multicopter=x_multicopter, controller=x_controller)
#     end
# end
