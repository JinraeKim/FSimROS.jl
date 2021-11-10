function Base.convert(multicopter::Multicopter, x)
    # Msg = pyimport("geometry_msgs.msg")
    Msg = pyimport("fsim_interfaces.msg")  # TODO
    @unpack p, v, R, ω = x
    quat = dcm_to_quat(SMatrix{3, 3}(R))
    # pose
    msg = Msg.PoseTwist()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat.q1, quat.q2, quat.q3, quat.q0  # be careful for the order
    # twist
    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v
    msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = ω
    msg
end

function Base.convert(multicopter::Multicopter, msg)
    Msg = pyimport("fsim_interfaces.msg")
    p = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    v = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
    quat = Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)  # be careful for the order
    R = quat_to_dcm(quat)
    ω = [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
    state = State(multicopter)(p, v, R, ω)
end
