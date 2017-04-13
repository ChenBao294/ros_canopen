/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <can_bridge/topic_to_canlib.h>
#include <can_interface/threading.h>
#include <can_interface/string.h>
#include <string>



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "topic_to_canlib_node");

  ros::NodeHandle nh(""), nh_param("~");

  unsigned int hardware_id, circuit_id;
  nh_param.param<unsigned int>("hardware_id", hardware_id, 0);
  nh_param.param<unsigned int>("circuit_id", circuit_id, 0);

  boost::shared_ptr<can::ThreadedCanlibInterface> driver = boost::make_shared<can::ThreadedCanlibInterface> ();

  if (!driver->init(hardware_id, circuit_id, 0))  // initialize device at hardware_id, circuit_id, 0 for no loopback.
  {
    ROS_FATAL("Failed to initialize can_device at hardware id %u, circuit id %u", hardware_id, circuit_id);
    return 1;
  }
    else
  {
    ROS_INFO("Successfully connected to hardware id %u, circuit id %u.", hardware_id, circuit_id);
  }

  socketcan_bridge::TopicToCanlib to_canlib_bridge(&nh, &nh_param, driver);
  to_canlib_bridge.setup();

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}
