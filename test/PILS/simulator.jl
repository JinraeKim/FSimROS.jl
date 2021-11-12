using PyCall
using FlightSims
using FSimROS
using UnPack

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
        timer_period = 1/500
        self.i = 0
        self.t = nothing
        self.time_received = false
        function timer_callback(self)
            x = copy(self.simulator.integrator.u)
            msg = state_to_msg(self.env.multicopter, x)
            self.publisher_.publish(msg)  # publish
            self.get_logger().info("time: $(self.t), state: $(x)")
            if self.control_received && self.t != nothing
                step_until!(self.simulator, self.t)
            end
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
        # env
        multicopter = LeeHexacopter()
        self.env = Multicopter_ZOH_Input(multicopter)
        state0 = State(self.env)()
        @unpack m, g = self.env.multicopter
        u0 = (m*g/6) * ones(6)  # initial control input (actually meaningless; for initialisation of simulator)
        self.simulator = Simulator(state0, Dynamics!(self.env), u0; tf=100)  # second
        # subscriber
        self.control_received = false
        function listener_callback(self, msg_control)
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
