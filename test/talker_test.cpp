/* Mit License
 * Copyright (C)
 * 2018 - Bhargav Dandamudi
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the 'Software'), to deal in the Software without
 * restriction, including without limitation the rights
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

/**
 * @file talker_test.cpp
 * @brief  To test if the service is working as expected
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2018-11-13
 */

#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include "beginner_tutorials/service.h"
#include "ros/ros.h"

/* --------------------------------------------------------------------------*/
/**
 * @brief  Test case to check if the string is getting updated or not
 */
/* --------------------------------------------------------------------------*/
TEST(TestSuite, ServiceTest) {
    ros::NodeHandle n;
    beginner_tutorials::service srv;
    ros::ServiceClient client =
        n.serviceClient<beginner_tutorials::service>("update_string");

    //  Tests that the service exists and updates the string as expected
    EXPECT_EQ(srv.response.outp, srv.request.inp);
}

/* --------------------------------------------------------------------------*/
/**
 * @brief To run tests which are declared above in TEST()
 *
 * @Param argc
 * @Param argv
 *
 * @Returns  RUN_ALL_TESTS
 */
/* -------------------------------------------------------------------------*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "talker_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
