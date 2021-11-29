using PyCall
using FlightSims
# using FSimBase
using FSimZoo
using FSimROS
using FSimPlots
using Plots
theme(:lime)
using UnPack
using OnlineStats
using DataFrames
using ComponentArrays


struct Multicopter_ZOH_Input <: AbstractEnv
    multicopter::Multicopter
end

function FSimBase.State(env::Multicopter_ZOH_Input)
    State(env.multicopter)
end

function FSimBase.Dynamics!(env::Multicopter_ZOH_Input)
    @Loggable function dynamics!(dx, x, input, t)
        @nested_log FSimZoo._Dynamics!(env.multicopter)(dx, x, nothing, t; u=input)
    end
end


struct ControllerEnv <: AbstractEnv
    controller::BacksteppingPositionController
    allocator::StaticAllocator
    multicopter::Multicopter  # TODO: not use the dummy multicopter; receive data from other nodes
end

function ControllerEnv(pos_cmd_func=nothing)
    multicopter = LeeHexacopter()
    @unpack m, B = multicopter
    controller = BacksteppingPositionController(m; pos_cmd_func=pos_cmd_func)
    allocator = PseudoInverseAllocator(B)
    env = ControllerEnv(controller, allocator, multicopter)
end

function FSimBase.State(env::ControllerEnv)
    @unpack controller, allocator, multicopter = env  # TODO: receive multicopter data from other nodes
    @unpack m, g = multicopter
    function state(; args_multicopter=())
        x_multicopter = State(multicopter)(args_multicopter...)
        pos0 = copy(x_multicopter.p)
        x_controller = State(controller)(pos0, m, g)
        ComponentArray(multicopter=x_multicopter, controller=x_controller)
    end
end

function FSimBase.Dynamics!(env::ControllerEnv)
    @unpack controller, allocator, multicopter = env
    @unpack m, J, g = multicopter
    @Loggable function dynamics!(dx, x, input, t; pos_cmd=nothing)
        @unpack p, v, R, ω = input
        # @unpack p, v, R, ω = x.multicopter
        @unpack ref_model, Td = x.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        νd, Ṫd, _... = Command(controller)(
                                           p, v, R, ω,
                                           xd, vd, ad, ȧd, äd, Td,
                                           m, J, g,
                                          )
        @nested_log :controller Dynamics!(controller)(dx.controller, x.controller, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        @nested_log :controller νd
    end
end

function plot_stat(x)
	fig_stat = plot()
        stat_E = MovingWindow(Float64, 500)
        stat_N = MovingWindow(Float64, 500)
        stat_U = MovingWindow(Float64, 500)
	fit!(stat_E, x.p[2])
	fit!(stat_N, x.p[1])
	fit!(stat_U, -x.p[3])
	fig_stat_E = plot(1:minimum([stat_E.b, stat_E.n]), stat_E.value; title="E [m]", label=nothing)
	fig_stat_N = plot(1:minimum([stat_N.b, stat_N.n]), stat_N.value; title="N [m]", label=nothing)
	fig_stat_U = plot(1:minimum([stat_U.b, stat_U.n]), stat_U.value; title="U [m]", label=nothing)
	fig_stat = plot(fig_stat_E, fig_stat_N, fig_stat_U; layout=(3, 1))
	fig_stat
end

function plot_traj(multicopter, x)
	fig_multicopter = plot()
	plot!(fig_multicopter, multicopter, x; xlim=(-2, 2), ylim=(-2, 2), zlim=(-1, 10))
	fig_multicopter
end

function plot_figs(multicopter, x)
	fig_multicopter = plot_traj(multicopter, x)
	fig_stat = plot_stat(x)
	# display(plot(fig_multicopter, fig_stat, layout=(1, 2)))
end


function main()
	t = 0.01
	multicopter = LeeHexacopter()
	env = Multicopter_ZOH_Input(multicopter)
	state0 = State(env)()
        @unpack m, g = env.multicopter
        u0 = (m*g/6) * ones(6)  # initial control input (actually meaningless; for initialisation of simulator)
        simulator = Simulator(state0, Dynamics!(env), u0; tf=100)  # second
        pos_cmd_func = t -> [sin(t), cos(t), 0.0]
        env_ctrl = ControllerEnv(pos_cmd_func)
        state0_ctrl = State(env_ctrl.multicopter)()  # dummy initial state of multicopter
        simulator_ctrl = Simulator(State(env_ctrl)(), Dynamics!(env_ctrl), state0_ctrl; tf=100)  # second

	# state update
	println("state update test...")
	step_until!(simulator, t)
	# control update
	println("control update test...")
	df = DataFrame()
	push!(simulator_ctrl, df)
	νd = df.sol[end].controller.νd  # virtual control
	step_until!(simulator_ctrl, t)

	# plot figures
	println("plotting test...")
	plot_figs(multicopter, state0)
end

main()
