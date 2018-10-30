/* Copyright (C)
 * 2018 - Bhargav Dandamudi
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */
/* --------------------------------------------------------------------------*/
/**
 * @copyright Copyright 2018 Bhargav Dandamudi
 *
 * @file talker.cpp
 * @author Bhargav Dandamudi
 * @brief ROS Publisher to send messages
 *
 */
/* ---------------------------------------------------------------------------*/
#include "ros/ros.h"
#include "std_msgs/String.h"

/* --------------------------------------------------------------------------*/
/**
 * @brief  The main function to creater talker node
 *
 * @Param argc is the number of input arguments
 * @Param argv is the publisher frequency, given as argument in command line
 *
 * @Returns 0
 */
/* --------------------------------------------------------------------------*/
int main(int argc, char **argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can
     * perform
     * any ROS arguments and name remapping that were provided at the command
     * line.
     * For programmatic remappings you can use a different version of init()
     * which takes
     * remappings directly, but for most command-line programs, passing argc and
     * argv is
     * the easiest way to do it.  The third argument to init() is the name of
     * the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "talker");

    /**
     * NodeHandle is the main access point to communications with the ROS
     * system.
     * The first NodeHandle constructed will fully initialize this node, and the
     * last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok()) {
        /**
         * This is a message object. You stuff it with data, and then publish
         * it.
         */
        std_msgs::String msg;

        std::stringstream ss;
        ss << "GO TERPS GO " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the
         * type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
