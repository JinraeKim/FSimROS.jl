function Base.convert(multicopter::Multicopter, x)
    @unpack p, v, R, ω = x
    quat = dcm_to_quat(SMatrix{3, 3}(R))
    # pose
    msg_pose = geoMsg.Pose()
    msg_pose.position.x, msg_pose.position.y, msg_pose.position.z = p
    msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z, msg_pose.orientation.w = quat.q1, quat.q2, quat.q3, quat.q0  # be careful for the order
    # twist
    msg_twist = geoMsg.Twist()
    msg_twist.linear.x, msg_twist.linear.y, msg_twist.linear.z = v
    msg_twist.angular.x, msg_twist.angular.y, msg_twist.angular.z = ω
    msg_pose, msg_twist
end

function Base.convert(multicopter::Multicopter, msg_pose, msg_twist)
    p = [msg_pose.position.x, msg_pose.position.y, msg_pose.position.z]
    v = [msg_twist.linear.x, msg_twist.linear.y, msg_twist.linear.z]
    quat = Quaternion(msg_pose.orientation.w, msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z)  # be careful for the order
    R = quat_to_dcm(quat)
    ω = [msg_twist.angular.x, msg_twist.angular.y, msg_twist.angular.z]
    state = State(multicopter)(p, v, R, ω)
end
