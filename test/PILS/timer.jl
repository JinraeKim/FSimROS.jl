using PyCall

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
std_msg = pyimport("std_msgs.msg")


@pydef mutable struct TimerNode <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "time")
	self.sent = false
        # publisher
        self.publisher_ = self.create_publisher(std_msg.Float64, "time", 10)
        timer_period = 0.001  # if it's too big, the simulation may diverge due to communication delay.
	dt = 0.001
	self.i = 0
	self.t = 0.0
	function timer_callback(self)
		msg = std_msg.Float64()
		msg.data = self.t
		self.publisher_.publish(msg)  # publish
		self.get_logger().info("time: $(self.t)")
		sleep(0.001)
		# if !self.sent
		# 	self.sent = true
		# 	sleep_time = 10
		# 	self.get_logger().info("sleep time: $(sleep_time)")
		# 	sleep(sleep_time)
		# end
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
