/* Copyright (C)
* 2018 - Bhargav Dandamudi
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the Software), to deal in the Software without
* restriction, including without limitation the rights
*
* to use, copy, modify, merge, publish, distribute,
* sublicense, and/or sell copies of the Software, and to permit
* persons to whom the Software is furnished to do so,subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall
* be included in all copies or substantial portions of the Software.
*
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
* ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM,OUT OF OR IN CONNECTION WITH
* THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
/* --------------------------------------------------------------------------*/
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <string>
#include "beginner_tutorials/service.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

std::string newString = "GO Terps GO !! ";   // NOLINT

/* --------------------------------------------------------------------------*/
/**
 * @brief  updates the string as given to service call argument
 *
 * @Param req as request of the service
 * @Param res as response to the service
 *
 * @Returns true
 */
/* -------------------------------------------------------------------------*/
bool update_string(beginner_tutorials::service::Request &req,      // NOLINT
                   beginner_tutorials::service::Response &res) {   // NOLINT
    newString = req.inp;    // "inp" is the input string in the service
    res.outp = newString;   // "outp" is the output string of the service
    ROS_INFO_STREAM("String is getting  updated");
    return true;
}
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
    // Create the service and advertise over ROS
    ros::ServiceServer service =
        n.advertiseService("update_string", update_string);

    // create a TransformBroadcaster object to send transformations
    static tf::TransformBroadcaster brdcstr;
    tf::Transform transform;

    // Publishing frequency is given as an argument in beginner.launch
    int freq;
    freq = std::atoi(argv[1]);   // give frequency the value of argument
                                 // ERROR Logging level check
    if (freq <= 0) ROS_ERROR_STREAM("Invalid publisher frequency");

    // DEBUG Logging level check
    ROS_DEBUG_STREAM("Publisher frequency set up to: " << freq);
    ROS_INFO_STREAM("Publisher frequency set up to: " << freq);

    ros::Rate loop_rate(freq);

    // If ROS node is not working
    if (!ros::ok()) ROS_FATAL_STREAM("ROS is node not running...");

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
        ss << newString << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        // WARN Logging level check
        if (freq < 5) ROS_WARN_STREAM("Frequency too low, Set it above 5Hz");

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the
         * type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        // create transformation and set the rotation
        transform.setOrigin(tf::Vector3(4, 4, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0.785, 0);
        transform.setRotation(q);

        // Broadcast the transformation
        brdcstr.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
