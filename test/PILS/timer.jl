using PyCall

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
std_msg = pyimport("std_msgs.msg")


@pydef mutable struct TimerNode <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "time")
	self.sent = [false, false, false]
        # publisher
        self.publisher_ = self.create_publisher(std_msg.Float64, "time", 10)
        timer_period = 1/500
        dt = 1/500
        self.i = 0
        self.t = 0.0
        function timer_callback(self)
            msg = std_msg.Float64()
            msg.data = self.t
	    # to avoid the communication delay due to JIT compilation
	    if self.sent == [false, false, false]
		    self.sent = [true, false, false]
	    elseif self.sent == [true, false, false]
		    self.sent = [true, true, false]
	    elseif self.sent == [true, true, false]
		    self.sent = [true, true, true]
		    sleep_time = 10
		    self.get_logger().info("sleep time: $(sleep_time) [s]. Wait a moment plz...")
		    sleep(sleep_time)
	    end
            self.publisher_.publish(msg)  # publish
            self.get_logger().info("time: $(self.t)")
            sleep(0.001)  # too fast publishing may yield divergence of simulation due to communication delay
            self.i = self.i + 1
            self.t = self.t + dt
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
    end
end


function main(args=nothing)
     rclpy.init(args=args)
     println("timer node started")
     timer_node = TimerNode()
     rclpy.spin(timer_node)
     timer_node.destroy_node()
     rclpy.shutdown()
end

main()
