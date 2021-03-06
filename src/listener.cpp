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

/**
 * @copyright Copyright 2018 Bhargav Dandamudi
 * @file listener.cpp
 * @brief
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2018-10-30
 */
#include "ros/ros.h"
#include "std_msgs/String.h"

int callbackCount = 0;
/* --------------------------------------------------------------------------*/
/**
 * @brief  Function to read messages and displays it
 *
 * @Param msg message recieved from talker node
 */
/* --------------------------------------------------------------------------*/
void chatterCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    ROS_DEBUG_STREAM("Callback accesed " << callbackCount << "times");
    callbackCount++;
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  To create Listener node
 *
 * @Param argc is the number of arguments
 * @Param argv is the arguments for frequency
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
    ros::init(argc, argv, "listener");

    /**
     * NodeHandle is the main access point to communications with the ROS
     * system.
     * The first NodeHandle constructed will fully initialize this node, and the
     * last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive
     * messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the
     * Subscriber
     * object go out of scope, this callback will automatically be unsubscribed
     * from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the
     * message
     * queue.  If messages are arriving faster than they are being processed,
     * this
     * is the number of messages that will be buffered up before beginning to
     * throw
     * away the oldest ones.
     */
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).
     * ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}
