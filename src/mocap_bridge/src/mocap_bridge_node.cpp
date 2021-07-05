// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <alessandro.fornasier@ieee.org>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "mocap_bridge.h"
#include <ros/ros.h>
#include <string>

// Main function
int main(int argc, char** argv) {

  // Launch ros node
  ros::init(argc, argv, "estimator_supervisor");
  ros::NodeHandle nh("~");
  ros::TransportHints().tcpNoDelay();

  ROS_INFO("Starting the Mocap Bridge");

  // Define parameters
  std::string subscriber_topic;
  std::string publisher_topic;

  // Parse parameters
  if(!nh.getParam("subscriber_topic", subscriber_topic)) {
    std::cout << std::endl;
    ROS_ERROR("No subscriber topic defined");
    std::exit(EXIT_FAILURE);
  }
  if(!nh.getParam("publisher_topic", publisher_topic)) {
    std::cout << std::endl;
    ROS_ERROR("No publisher topic defined");
    std::exit(EXIT_FAILURE);
  }

  // Instanciate the supervisor
  MocapBridge MocapBridge(nh, subscriber_topic, publisher_topic);

  // Spin
  ros::spin();

  // Done!
  std::cout << std::endl;
  ROS_INFO("Done!");
  return EXIT_SUCCESS;

}

