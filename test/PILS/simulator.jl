using PyCall
using FlightSims
using FSimROS
# using Plots
# theme(:lime)
# using FSimPlots
using UnPack
# using OnlineStats

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
fsim_msg = pyimport("fsim_interfaces.msg")
std_msg = pyimport("std_msgs.msg")


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


@pydef mutable struct StateNode <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "state_node")
        # publisher
        self.publisher_ = self.create_publisher(fsim_msg.PoseTwist, "state", 5)
        timer_period = 0.0001
	self.i = 0
	self.t = nothing
	self.time_received = false
        # self.fig = plot()  # it might not be a good idea to run plotting function in StateNode for latency
	# self.stat = Group(Partition(KHist(25)), Partition(KHist(25)), Partition(KHist(25)))
	# self.plot_stat = false
        function timer_callback(self)
		x = copy(self.simulator.integrator.u)
            msg = state_to_msg(self.env.multicopter, x)
            self.publisher_.publish(msg)  # publish
	    # if self.i % 1000 == 0
		    # self.get_logger().info("time: $(self.t)")
		    self.get_logger().info("time: $(self.t), state: $(x)")
	    # end
            if self.control_received && self.t != nothing
		    # self.plot_stat = true
		    # step_until!(self.simulator, self.simulator.integrator.t + timer_period)
		    step_until!(self.simulator, self.t)
            end
	    #self.i = self.i + 1
	    #if self.i % 100 == 0  # too frequent plotting may yield divergence of simulation due to communication delay
	    ## if false
	    #        # plotting multicopter
	    #        # plot!(self.fig, self.env.multicopter, copy(self.simulator.integrator.u); xlim=(-2, 2), ylim=(-2, 2), zlim=(-1, 10))
	    #        # display(self.fig)
	    #        # self.fig = plot()
	    #end
	    ## if self.plot_stat
	    #if true
	    #        # plotting stats
	    #        fit!(self.stat, zip(x.p...))
	    #        display(plot(self.stat; layout=(3, 1)))
	    #end
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
        # env
	multicopter = LeeHexacopter()
        self.env = Multicopter_ZOH_Input(multicopter)
        state0 = State(self.env)()
	@unpack m, g = self.env.multicopter
	u0 = (m*g/6) * ones(6)  # initial control input
        self.simulator = Simulator(state0, Dynamics!(self.env), u0; tf=100)  # second
        # subscriber
        self.control_received = false
        function listener_callback(self, msg_control)
            # @show self.control_received
            if !self.control_received
                self.control_received = true
            end
            input = [msg_control.u1, msg_control.u2, msg_control.u3, msg_control.u4, msg_control.u5, msg_control.u6]
            self.simulator.integrator.p = input
        end
	function listener_callback_time(self, msg)
		t = msg.data
		if !self.time_received
			self.time_received = true
			self.simulator.integrator.t = t
		end
		self.t = t
	end
        self.subscription = self.create_subscription(fsim_msg.RotorRateHexa, "control", msg -> listener_callback(self, msg), 10)
        self.subscription_time = self.create_subscription(std_msg.Float64, "time", msg -> listener_callback_time(self, msg), 10)
    end
end

function main(args=nothing)
    rclpy.init(args=args)
    println("state node started")
    state_node = StateNode()
    rclpy.spin(state_node)
    state_node.destroy_node()
    rclpy.shutdown()
end

main()
