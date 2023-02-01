#!/usr/bin/env python3

import rospy
from pedsim_msgs.msg import AgentStates


def callback(msg):
    rospy.loginfo("Received a /pedsim_simulator/simulated_agents message!")
    rospy.loginfo("Number of Agents: %d\n" % len(msg.agent_states))
    for agent in msg.agent_states:
        rospy.loginfo("Agent id/type: [%d/%d]" % (agent.id, agent.type))
        rospy.loginfo("Agent Position: [%f, %f, %f]" % (agent.pose.position.x, agent.pose.position.y,
                                                        agent.pose.position.z))
        rospy.loginfo("Agent Orientation: [%f, %f, %f, %f]" % (agent.pose.orientation.x, agent.pose.orientation.y,
                                                               agent.pose.orientation.z, agent.pose.orientation.w))
        rospy.loginfo("Agent Speed Linear: [%f, %f, %f]" % (agent.twist.linear.x, agent.twist.linear.y,
                                                            agent.twist.linear.z))
        rospy.loginfo("Agent Speed Angular: [%f, %f, %f]\n" % (agent.twist.angular.x, agent.twist.angular.y,
                                                               agent.twist.angular.z))
        #print("Agent id/type: [%d/%d]" % (agent.id, agent.type))
    return msg


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('custom_listener', anonymous=True)

    rospy.Subscriber('pedsim_simulator/simulated_agents', AgentStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
