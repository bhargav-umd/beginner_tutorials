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
