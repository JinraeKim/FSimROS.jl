using PyCall
using FlightSims
using FSimROS
using Plots
theme(:lime)
using FSimPlots
using UnPack
using OnlineStats

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
fsim_msg = pyimport("fsim_interfaces.msg")
std_msg = pyimport("std_msgs.msg")


@pydef mutable struct StateNode <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "viz_node")
        # publisher
	self.i = 0
	self.t = nothing
	self.time_received = false
        self.fig_multicopter = plot()  # it might not be a good idea to run plotting function in StateNode for latency
	self.fig_stat = plot()
	self.stat_p_x = Partition(KHist(10))
	self.stat_p_y = Partition(KHist(10))
	self.stat_p_z = Partition(KHist(10))
        # env
	self.multicopter = LeeHexacopter()
        # subscriber
        self.control_received = false
        function listener_callback(self, msg_state)
		if self.t != nothing
			x = msg_to_state(self.multicopter, msg_state)
			self.i = self.i + 1
			# if false
			# plotting multicopter
			self.fig_multicopter = plot()
			plot!(self.fig_multicopter, self.multicopter, x; xlim=(-2, 2), ylim=(-2, 2), zlim=(-1, 10))
			# plotting stats
			fit!(self.stat_p_x, x.p[1])
			fit!(self.stat_p_y, x.p[2])
			fit!(self.stat_p_z, x.p[3])
			fig_stat_p_x = plot(self.stat_p_x)
			fig_stat_p_y = plot(self.stat_p_y)
			fig_stat_p_z = plot(self.stat_p_z)
			self.fig_stat = plot(fig_stat_p_x, fig_stat_p_y, fig_stat_p_z; layout=(3, 1))
		end
		display(plot(self.fig_multicopter, self.fig_stat, layout=(1, 2)))
        end
	function listener_callback_time(self, msg)
		t = msg.data
		if !self.time_received
			self.time_received = true
		end
		self.t = t
	end
        self.subscription = self.create_subscription(fsim_msg.PoseTwist, "state", msg -> listener_callback(self, msg), 10)
        self.subscription_time = self.create_subscription(std_msg.Float64, "time", msg -> listener_callback_time(self, msg), 10)
    end
end

function main(args=nothing)
    rclpy.init(args=args)
    println("viz node started")
    state_node = StateNode()
    rclpy.spin(state_node)
    state_node.destroy_node()
    rclpy.shutdown()
end

main()
