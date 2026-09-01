/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*         					  ROS2 Driver			  			*/
/*          Copyright 2023, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "adnav_driver.h"

int main(int argc, char * argv[])
{
	// Initialize ros2
	rclcpp::init(argc, argv);

	// create an multithreaded executor
	// ADV-153: explicit small thread count instead of the rclcpp default of
	// std::thread::hardware_concurrency(), which over-provisions massively on many-core
	// machines (28 threads observed on a 28-core bench host) for a node with only
	// reading_group_/publishing_group_/service_group_ (3 callback groups) worth of real
	// concurrency needs.
	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);

	// Create a shared pointer to the driver node.
	// ADV-153: the constructor performs the device handshake (waitForDevicePacket()), which now
	// throws std::runtime_error on a bounded timeout instead of spinning forever - catch that here
	// so a disconnected/misconfigured device produces a clean, loud failure instead of a hang.
	std::shared_ptr<rclcpp::Node> node;
	try {
		node = std::make_shared<adnav::Driver>();
	} catch (const std::exception & e) {
		RCLCPP_FATAL(rclcpp::get_logger("adnav_driver"), "Failed to start driver: %s", e.what());
		rclcpp::shutdown();
		return 1;
	}

	// Add the driver node to the executor and spin it.
	executor.add_node(node);

	while(rclcpp::ok()) {
		executor.spin();
	}

	rclcpp::shutdown();
  	return 0;
}