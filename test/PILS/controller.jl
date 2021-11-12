using PyCall
using FlightSims
using FSimROS
using DataFrames
using UnPack
using ComponentArrays

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
fsim_msg = pyimport("fsim_interfaces.msg")
std_msg = pyimport("std_msgs.msg")


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


@pydef mutable struct ControlNode <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "control_node")
        # publisher
        self.publisher_ = self.create_publisher(fsim_msg.RotorRateHexa, "control", 5)
        timer_period = 1/500
        self.i = 0
        self.t = nothing
        self.time_received = false
        function timer_callback(self)
            msg_control = fsim_msg.RotorRateHexa()
            # TODO: put in your custom control law
            # u_cmd = (m*g/6)*ones(6)  # hovering input
            df = DataFrame()
            push!(self.simulator, df)
            νd = df.sol[end].controller.νd  # virtual control
            @unpack m, g = self.env.multicopter
            u_cmd = Command(self.env.allocator)(νd)
            msg_control.u1 = u_cmd[1]
            msg_control.u2 = u_cmd[2]
            msg_control.u3 = u_cmd[3]
            msg_control.u4 = u_cmd[4]
            msg_control.u5 = u_cmd[5]
            msg_control.u6 = u_cmd[6]
            self.publisher_.publish(msg_control)  # publish
            self.get_logger().info("time: $(self.t), control: $(u_cmd)")
            # end
            self.i = self.i + 1
            if self.state_multicopter_received && self.t != nothing
                step_until!(self.simulator, self.t)
            end
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
        # env
        pos_cmd_func = t -> [sin(t), cos(t), 0.0]
        self.env = ControllerEnv(pos_cmd_func)
        x0 = State(self.env.multicopter)()  # dummy initial state of multicopter
        self.simulator = Simulator(State(self.env)(), Dynamics!(self.env), x0; tf=100)  # second
        # subscriber
        self.state_multicopter_received = false
        function listener_callback(self, msg_state)
            if !self.state_multicopter_received
                self.state_multicopter_received = true
            end
            self.simulator.integrator.p = msg_to_state(self.env.multicopter, msg_state)
        end
        function listener_callback_time(self, msg)
            t = msg.data
            if !self.time_received
                self.time_received = true
                self.simulator.integrator.t = t
            end
            self.t = t
        end
        self.subscription = self.create_subscription(fsim_msg.PoseTwist, "state", msg -> listener_callback(self, msg), 10)
        self.subscription_time = self.create_subscription(std_msg.Float64, "time", msg -> listener_callback_time(self, msg), 10)
    end
end

function main(args=nothing)
    rclpy.init(args=args)
    println("control node started")
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()
end

main()
