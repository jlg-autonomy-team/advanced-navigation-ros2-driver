/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*         					  ROS2 Driver			  			*/
/*          Copyright 2024, Advanced Navigation Pty Ltd         */
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

namespace adnav {
/**
 * @brief Constructor for the Advanced Navigation Driver node
 *
 * This will create a Driver instance. Declare node parameters, setup threading groups and callbacks.
 * Initialize the log file, and open the communications to the device.
 */
Driver::Driver(): rclcpp::Node("adnav_driver"), msg_write_done_(false)
{
	// Set default values
	memset(&acknowledge_packet_, -1, sizeof(acknowledge_packet_));

	// ~~~~~~~~~~ Create the callback groups
	// Callback group only for reading ANPP Packets
	reading_group_ 		= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	// Multithreaded group for publishing ROS messages
	publishing_group_	= this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

	// Group for completing incoming services.
	service_group_		= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	// Setup parameters for the node
	setupParamService();

	// Get the name of the node
	node_name_ = this->get_name();
	RCLCPP_INFO(this->get_logger(), "\nNamespace: %s\n", node_name_.c_str());

	// Create timers for callbacks
	publish_timer_ = this->create_wall_timer(
      	publish_timer_interval_ , std::bind(&Driver::publishTimerCallback, this), publishing_group_);

	read_timer_ = this->create_wall_timer(
		read_timer_interval_, std::bind(&Driver::recievePackets, this), reading_group_);

	// Setup Services for the node
	createServices();

	// Open a new ANPP Log file for data logging
	anpp_logger_.openFile("Log_", ".anpp", log_path_);

	// Open Communications with the device
	communicator_ = std::make_unique<adnav::Communicator>(comms_data_);
	communicator_->open();

	// Request device info with Packet 1 and 3
	waitForDevicePacket();

	// Create Publishers for the node
	createPublishers();

	// Send current setup of timer period, and packet periods to the device.
	deviceSetup();

	RCLCPP_INFO(this->get_logger(), "Your Advanced Navigation ROS driver is currently running\nPress Ctrl-C to interrupt\n");
}

/**
 * @brief Deconstructor for the Node
 *
 * Closes the Logfiles and gives name to the user. Shutsdown the communication method
 */
Driver::~Driver() {
	RCLCPP_INFO(this->get_logger(), "Destructing Adnav_Driver node");

	anpp_logger_.closeFile();

	// if the Ntrip client is initialised and running or the logger is open.
	if (ntrip_client_.get() != nullptr && (ntrip_client_->service_running() || rtcm_logger_.is_open())) {
		ntrip_client_->stop();

		// Close the logfile
		rtcm_logger_.closeFile();
	}

	communicator_->close();
}

/**
 * @brief Function to ask for device information from a Advanced navigation device and wait for its response.
 *
 * ADV-153: bounded with a timeout + backoff instead of spinning forever. Confirmed on real
 * hardware that a baud-rate/cabling mismatch left this looping at ~100% CPU indefinitely with
 * no diagnostic, since the device never responds. Now: (1) sleeps briefly when no bytes are
 * available instead of tight-looping the read() call, (2) only re-sends the request packet
 * periodically instead of every single iteration, (3) throws after DEVICE_HANDSHAKE_TIMEOUT_SEC
 * with a clear error so the caller (the constructor) fails fast and loud instead of hanging.
 */
void Driver::waitForDevicePacket() {
	// initialize the decoder.
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	an_decoder_initialise(&an_decoder);
	bool recieved = false;
	int bytes_received;

	RCLCPP_DEBUG(this->get_logger(), "Requesting Device Info");

	const auto handshake_start = std::chrono::steady_clock::now();
	// ADV-153: initialise to "now - interval" (not time_point::min()) so the very first loop
	// iteration's `now - last_request_time >= interval` check is guaranteed to be true and
	// actually fire the first request. Using time_point::min() here overflowed the duration
	// subtraction (undefined behaviour) and silently made the check false forever, so
	// requestDeviceInfo() was never called - confirmed on hardware: a manual, correctly-framed
	// ANPP request via a raw serial connection got an immediate response, but this loop's own
	// request was never actually sent.
	auto last_request_time = handshake_start - std::chrono::milliseconds(DEVICE_HANDSHAKE_REQUEST_INTERVAL_MS);

	while(recieved == false && rclcpp::ok()) {
		const auto now = std::chrono::steady_clock::now();

		if (now - handshake_start >= std::chrono::seconds(DEVICE_HANDSHAKE_TIMEOUT_SEC)) {
			RCLCPP_ERROR(this->get_logger(),
				"Timed out after %d seconds waiting for a Device Information Packet (ANPP.3) response. "
				"Check that the device is powered, the cable/port (%s) is correct, and that baud_rate "
				"(%d) matches the device's actual configured baud rate.",
				DEVICE_HANDSHAKE_TIMEOUT_SEC, comms_data_.com_port.c_str(), comms_data_.baud_rate);
			throw std::runtime_error("Timed out waiting for device handshake (Device Information Packet)");
		}

		// Only re-send the request packet periodically rather than every loop iteration -
		// the device only needs to see it once per interval, not hundreds of times per second.
		if (now - last_request_time >= std::chrono::milliseconds(DEVICE_HANDSHAKE_REQUEST_INTERVAL_MS)) {
			requestDeviceInfo();
			last_request_time = now;
		}

		// Read in some data from the connection.
		bytes_received = communicator_->read(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder));

		// Decode all data and act on only the device info data.
		if (bytes_received > 0)
		 {
			anpp_logger_.writeAndIncrement((char*) an_decoder_pointer(&an_decoder), bytes_received);

			// Increment the decode buffer length by the number of bytes received
			an_decoder_increment(&an_decoder, bytes_received);

			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			 {
				RCLCPP_DEBUG(this->get_logger(), "[WaitForDevicePacket]ID: %d", an_packet->id);

				if(an_packet->id == packet_id_device_information) {
					RCLCPP_DEBUG(this->get_logger(), "Received Device Information Packet (ANPP.3)");
					deviceInfoDecoder(an_packet);
					recieved = true;

				}

				// Ensure that you free the an_packet when your done with it or you will leak memory
				an_packet_free(&an_packet);
			}
		} else {
			// No data available yet - avoid busy-spinning the CPU while waiting.
			std::this_thread::sleep_for(std::chrono::milliseconds(DEVICE_HANDSHAKE_POLL_SLEEP_MS));
		}
	}
}

/**
 * @brief Function to request a Advanced Navigation Devices info using ANPP.
 */
void Driver::requestDeviceInfo() {
	an_packet_t* an_packet;

	an_packet = encode_request_packet(packet_id_device_information);
	encodeAndSend(an_packet);
}

/**
 * @brief Function to declare topics and publishers for the adnav_driver node.
 */
void Driver::createPublishers() {
	// Creating the ROS2 Publishers
	imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(std::string(node_name_ + "/imu"), 10);
	imu_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(std::string(node_name_ + "/imu_raw"), 10);
	nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(std::string(node_name_ + "/nav_sat_fix"), 10);
	magnetic_field_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(std::string(node_name_ + "/magnetic_field"), 10);
	barometric_pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(std::string(node_name_ + "/barometric_pressure"), 10);
	temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(std::string(node_name_ + "/temperature"), 10);
	// ADV-153 Phase 3: DEPRECATED topic - see twist_msg_ in adnav_driver.h for why.
	twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(std::string(node_name_ + "/twist"), 10);
	pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(std::string(node_name_ + "/pose"), 10);
	system_status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(std::string(node_name_ + "/system_status"), 10);
	filter_status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(std::string(node_name_ + "/filter_status"), 10);

	// ADV-153 Phase 3: lightweight per-packet raw topics (packets 24/25/26/35/38/39/40/42/43)
	position_std_dev_pub_ = this->create_publisher<adnav_interfaces::msg::PositionStdDev>(std::string(node_name_ + "/position_std_dev"), 10);
	velocity_std_dev_pub_ = this->create_publisher<adnav_interfaces::msg::VelocityStdDev>(std::string(node_name_ + "/velocity_std_dev"), 10);
	euler_std_dev_pub_ = this->create_publisher<adnav_interfaces::msg::EulerStdDev>(std::string(node_name_ + "/euler_std_dev"), 10);
	ned_velocity_pub_ = this->create_publisher<adnav_interfaces::msg::NedVelocity>(std::string(node_name_ + "/ned_velocity"), 10);
	body_acceleration_pub_ = this->create_publisher<adnav_interfaces::msg::BodyAcceleration>(std::string(node_name_ + "/body_acceleration"), 10);
	euler_orientation_pub_ = this->create_publisher<adnav_interfaces::msg::EulerOrientation>(std::string(node_name_ + "/euler_orientation"), 10);
	quaternion_orientation_pub_ = this->create_publisher<adnav_interfaces::msg::QuaternionOrientation>(std::string(node_name_ + "/quaternion_orientation"), 10);
	angular_velocity_pub_ = this->create_publisher<adnav_interfaces::msg::AngularVelocity>(std::string(node_name_ + "/angular_velocity"), 10);
	angular_acceleration_pub_ = this->create_publisher<adnav_interfaces::msg::AngularAcceleration>(std::string(node_name_ + "/angular_acceleration"), 10);
}

/**
 * @brief Function to declare and assign callbacks to services for the adnav_driver node.
 */
void Driver::createServices() {
	packet_period_srv_ = this->create_service<adnav_interfaces::srv::PacketPeriods>(
		(node_name_ + "/packet_periods"),
		std::bind(
			&Driver::srvPacketPeriods,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		rmw_qos_profile_services_default,
		service_group_);

	packet_period_timer_srv_ = this->create_service<adnav_interfaces::srv::PacketTimerPeriod>(
		(node_name_ + "/packet_timer_period"),
		std::bind(
			&Driver::srvPacketTimerPeriod,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		rmw_qos_profile_services_default,
		service_group_);

	request_packet_srv_ = this->create_service<adnav_interfaces::srv::RequestPackets>(
		(node_name_ + "/request_packet"),
		std::bind(
			&Driver::srvRequestPackets,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		rmw_qos_profile_services_default,
		service_group_);

	ntrip_srv_ = this->create_service<adnav_interfaces::srv::Ntrip>(
		(node_name_ + "/ntrip"),
		std::bind(
			&Driver::srvNtrip,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		rmw_qos_profile_services_default,
		service_group_);
}

/**
 * @brief Method to push requested configuration to the device.
 *
 * This method will take stored data on the configurationand form configuration packets which
 * will then be pushed to the device.
 */
void Driver::deviceSetup() {
	RCLCPP_INFO(this->get_logger(), "Sending requested configuration to device:");

	// Create and send a Packet Timer Period Packet.
	acknowledge_recieve_ = true; // Since we are not waiting for device acknowledgement at startup
	(void)SendPacketTimer(packet_timer_period_); // Will overwrite acknowledge_receive_ to false
	// ADV-153: SendPacketTimer()'s built-in AcknowledgeHandler() wait is a no-op this early in the
	// constructor - nothing decodes incoming bytes until the executor starts spinning
	// read_timer_'s callback, well after this function returns - so it always instantly returned
	// stale/meaningless data. Confirmed on real hardware: this bench unit's
	// packet_timer_period=1000 was silently REJECTED (Acknowledge result=3/failure_range) for
	// most of a session before anyone noticed, purely because nothing ever checked. Manually pump
	// a short, bounded read/decode loop here instead to actually catch and log the real result.
	pumpAndLogAcknowledge("Packet Timer Period");

	// Create and fill a periods format.
	std::vector<adnav_interfaces::msg::PacketPeriod> packet_periods;
	for(uint64_t i = 0; (i+i) < packet_request_.size(); i++) {
		adnav_interfaces::msg::PacketPeriod period;
		period.packet_id = packet_request_[i+i];
		period.packet_period = packet_request_[i+i+1];
		packet_periods.push_back(period);
	}

	// Since we are not waiting for device acknowledgement at startup
	acknowledge_recieve_ = true;
	(void)SendPacketPeriods(packet_periods); // Will overwrite acknowledge_receive_ to false.
	pumpAndLogAcknowledge("Packet Periods");
}

/**
 * @brief Manually pump a short, bounded read/decode loop to catch and log the real Acknowledge
 * Packet result for a config packet just sent from deviceSetup().
 *
 * Needed only for the deviceSetup() call site: it runs synchronously in the constructor, before
 * the executor (and therefore read_timer_'s callback) starts spinning, so there is nothing else
 * pumping communicator_->read()/decodePackets() to receive the device's response during that
 * window. Reuses decodePackets() (which dispatches Acknowledge Packets to acknowledgeDecoder())
 * so the check-and-log logic here doesn't have to duplicate any ANPP parsing.
 *
 * @param label human-readable name of the config just sent, for the log message.
 */
void Driver::pumpAndLogAcknowledge(const std::string& label) {
	an_decoder_t an_decoder;
	an_decoder_initialise(&an_decoder);
	acknowledge_recieve_ = false;

	const auto start = std::chrono::steady_clock::now();
	while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
		int bytes_received = communicator_->read(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder));
		if (bytes_received > 0) {
			decodePackets(an_decoder, bytes_received);
			if (acknowledge_recieve_) {
				break;
			}
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	if (!acknowledge_recieve_) {
		RCLCPP_WARN(this->get_logger(), "%s: no Acknowledge Packet received within 500ms of sending.",
			label.c_str());
		return;
	}
	if (acknowledge_packet_.acknowledge_result == acknowledge_success) {
		RCLCPP_INFO(this->get_logger(), "%s: accepted by device.", label.c_str());
	} else {
		RCLCPP_ERROR(this->get_logger(),
			"%s: REJECTED by device (Acknowledge result=%d - see acknowledge_result_e in "
			"ins_packets.h, e.g. 3=failure_range means a requested value is out of range for this "
			"specific unit even if within the generic ANPP-documented bounds). The device is still "
			"running its previous, last-successfully-accepted configuration.",
			label.c_str(), acknowledge_packet_.acknowledge_result);
	}
}

/**
 * @brief Method to setup node parameters, callbacks and event handlers
 *
 * This method will initialize the event handlers and callbacks for parameter updates,
 * as well as declaring, describing, and assigning values to node parameters.
 * Will assign a default value if one is not provided already.
 */
void Driver::setupParamService() {

	// Setup and validate launch parameters.
	setupParams();

	// Register the callback for parameter changes.
	// Will be called whenever a parameter is update is requested.
	param_set_cb_ = this->add_on_set_parameters_callback(
            std::bind(&Driver::ParamSetCallback,
					this,
					std::placeholders::_1));
	// Set Parameter Event Handler will call attached callbacks when verification is completed.
	param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

	// publish period updater callback
	publish_us_cb_ = param_handler_->add_parameter_callback("publish_us",
			std::bind(&Driver::updatePublishUs,
					this,
					std::placeholders::_1));

	// read period updater callback
	read_us_cb_ = param_handler_->add_parameter_callback("read_us",
			std::bind(&Driver::updateReadUs,
					this,
					std::placeholders::_1));

	// packet request updater callback
	packet_request_cb_ = param_handler_->add_parameter_callback("packet_request",
			std::bind(&Driver::updatePacketRequest,
					this,
					std::placeholders::_1));

	// packet timer updater callback
	packet_timer_cb_ = param_handler_->add_parameter_callback("packet_timer_period",
			std::bind(&Driver::updatePacketTimer,
					this,
					std::placeholders::_1));
}

/**
 * @brief Method to setup node parameters.
 *
 * This method will declare, describe, and assign values to node parameters
 * Will assign a default value if one is not provided already, or a provided value fails verification.
 */
void Driver::setupParams() {
	std::stringstream ss;

	// Baud Rate - Read only
	rcl_interfaces::msg::ParameterDescriptor baud_description = rcl_interfaces::msg::ParameterDescriptor();
	ss << 	"Baud rate for communication with AdvancedNavigation Device\n" <<
			"Default: " << DEFAULT_BAUD_RATE;
	baud_description.description = ss.str();
	ss.str(""); // empty the stream
	baud_description.name = "baud_rate";
	baud_description.read_only = true;
	baud_description.additional_constraints = "\n\tSupported Baud Rates:\n\t  2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600, 1000000, 2000000";
	this->declare_parameter<int>("baud_rate", DEFAULT_BAUD_RATE, baud_description);
	if( validateBaudRate( this->get_parameter("baud_rate") ).successful ) { // Check that it is valid and save
		comms_data_.baud_rate = (int) this->get_parameter("baud_rate").as_int();
	}else {// If invalid save default.
		RCLCPP_ERROR(this->get_logger(), "Invalid baud rate. Setting to default: %d", DEFAULT_BAUD_RATE);
		this->set_parameter(rclcpp::Parameter("baud_rate", DEFAULT_BAUD_RATE));
		comms_data_.baud_rate = DEFAULT_BAUD_RATE;
	}

	// Com_port - Read only
	rcl_interfaces::msg::ParameterDescriptor com_port_description = rcl_interfaces::msg::ParameterDescriptor();
	com_port_description.description = "Communications port for serial operation\n  Default: \"/dev/ttyUSB0\"";
	com_port_description.name = "com_port";
	com_port_description.read_only = true;
	this->declare_parameter<std::string>("com_port", DEFAULT_COM_PORT, com_port_description);
	if( validateComPort( this->get_parameter("com_port") ).successful ) { // Check that it is valid and save
		comms_data_.com_port = this->get_parameter("com_port").as_string();
	}else { // If invalid save default.
		RCLCPP_ERROR(this->get_logger(), "Invalid com port. Setting to default: %s", DEFAULT_COM_PORT);
		this->set_parameter(rclcpp::Parameter("com_port", DEFAULT_COM_PORT));
		comms_data_.com_port = DEFAULT_COM_PORT;
	}

	rcl_interfaces::msg::IntegerRange intervalRange;
	intervalRange.from_value = 1000;
	intervalRange.to_value   = 1000000;
	intervalRange.step 		 = 1;

	// Read timer interval
	rcl_interfaces::msg::ParameterDescriptor read_description = rcl_interfaces::msg::ParameterDescriptor();
	ss << 	"Parameter for controlling sensor serial polling period in microseconds (μs)\n" <<
			"  Default: " << DEFAULT_TIMER_PERIOD << "μs";
	read_description.name = "read_us";
	read_description.description = ss.str();
	ss.str(""); // empty the stream
	read_description.integer_range.push_back(intervalRange);
	this->declare_parameter<int>("read_us", DEFAULT_TIMER_PERIOD, read_description);
	if( validateReadUs( this->get_parameter("read_us") ).successful ) { // Check that it is valid and save
		read_timer_interval_ = std::chrono::microseconds(this->get_parameter("read_us").as_int());
	}else { // If invalid save default.
		RCLCPP_ERROR(this->get_logger(), "Invalid period. Setting to default: %d", DEFAULT_TIMER_PERIOD);
		this->set_parameter(rclcpp::Parameter("read_us", DEFAULT_TIMER_PERIOD));
		read_timer_interval_ = std::chrono::microseconds(DEFAULT_TIMER_PERIOD);
	}

	// Publish timer interval
	rcl_interfaces::msg::ParameterDescriptor publish_description = rcl_interfaces::msg::ParameterDescriptor();
	ss << 	"Parameter for controlling ROS publishing period in microseconds (μs)\n" <<
			"  Default: " << DEFAULT_TIMER_PERIOD << "μs";
	publish_description.description = ss.str();
	ss.str(""); // empty the stream
	publish_description.name = "publish_us";
	publish_description.integer_range.push_back(intervalRange);
	publish_description.additional_constraints = "Value must be greater than read_us";
	this->declare_parameter<int>("publish_us", DEFAULT_TIMER_PERIOD, publish_description);
	if( validatePublishUs( this->get_parameter("publish_us") ).successful ) { // Check that it is valid and save
		publish_timer_interval_ = std::chrono::microseconds(this->get_parameter("publish_us").as_int());
	}else { // If invalid save default.
		// Is the current read_us higher than the default set it to read_us
		if(DEFAULT_TIMER_PERIOD < this->get_parameter("read_us").as_int()) {
			RCLCPP_ERROR(this->get_logger(), "Invalid period. Setting to read_us: %ld", this->get_parameter("read_us").as_int());
			this->set_parameter(rclcpp::Parameter("publish_us", this->get_parameter("read_us").as_int()));
			publish_timer_interval_ = std::chrono::microseconds(this->get_parameter("read_us").as_int());
		}else { // otherwise set to default.
			RCLCPP_ERROR(this->get_logger(), "Invalid period. Setting to default: %d", DEFAULT_TIMER_PERIOD);
			this->set_parameter(rclcpp::Parameter("publish_us", DEFAULT_TIMER_PERIOD));
			publish_timer_interval_ = std::chrono::microseconds(DEFAULT_TIMER_PERIOD);
		}
	}



	// Request packet array
	rcl_interfaces::msg::ParameterDescriptor packet_request_description = rcl_interfaces::msg::ParameterDescriptor();
	ss << 	"\tInteger Vector for Requested Packet ID's and their Period\n" <<
			"\t\tSee ANPP Documentation for more information on packets and IDs and Periods\n" <<
			"  Usage: [ID1, Period1, ID2, Period2, ...]\n" <<
			"  Default: [20, 10, 28, 10]";
	packet_request_description.description = ss.str();
	ss.str(""); // empty the stream
	packet_request_description.name = "packet_request";
	ss << 	"Must contain a minimum of 1 packet, every packet ID must have a corresponding rate.\n"<<
			"\tFor IDs:\n" <<
			"\t  Status Packets: [0-" << end_system_packets-1 << "]\n" <<
			"\t  State Packets: [" << START_STATE_PACKETS << "-" << end_state_packets-1 << "]\n" <<
			"\t  Configuration Packets: [" << START_CONFIGURATION_PACKETS << "-" << end_configuration_packets-1 << "]\n" <<
			"\tFor Periods:\n" <<
			"\t  Min value: " << MIN_PACKET_PERIOD << "\n" <<
			"\t  Max value: " << MAX_PACKET_PERIOD << "\n" <<
			"\t  Step: " << 1;
	packet_request_description.additional_constraints = ss.str();
	ss.str(""); // empty the stream
	std::vector<int64_t> default_vector(DEFAULT_PACKET_REQUEST, DEFAULT_PACKET_REQUEST + (sizeof(DEFAULT_PACKET_REQUEST)/sizeof(DEFAULT_PACKET_REQUEST[0])));
	this->declare_parameter("packet_request", rclcpp::ParameterValue(default_vector), packet_request_description);
	if( validatePacketRequest( this->get_parameter("packet_request") ).successful ) { // Check that it is valid and save
		packet_request_ = this->get_parameter("packet_request").as_integer_array();
	}else { // If invalid save default.
		RCLCPP_ERROR(this->get_logger(), "Invalid packet request. Setting to default: %s", DEFAULT_PACKET_REQUEST_STR);
		this->set_parameter(rclcpp::Parameter("packet_request", default_vector));
		packet_request_ = default_vector;
	}

	// Packet Timer Period
	rcl_interfaces::msg::ParameterDescriptor packet_timer_period_description = rcl_interfaces::msg::ParameterDescriptor();
	ss << 	"\tInteger value for Packet Timing Period in μs\n"<<
			"\t\tSee ANPP Documentation for more information on Packets and IDs\n" <<
			"  Default: 10,000";
	packet_timer_period_description.description = ss.str();
	ss.str(""); // empty the stream
	packet_timer_period_description.name = "packet_timer_period";
	rcl_interfaces::msg::IntegerRange ptpdRange;
	ptpdRange.from_value = MIN_TIMER_PERIOD;
	ptpdRange.to_value   = MAX_TIMER_PERIOD;
	ptpdRange.step 		 = 1;
	packet_timer_period_description.integer_range.push_back(ptpdRange);
	this->declare_parameter<int>("packet_timer_period", 10000, packet_timer_period_description);
	if( validatePacketTimer(this->get_parameter("packet_timer_period") ).successful) { // Check that it is valid and save
		packet_timer_period_ = (int) this->get_parameter("packet_timer_period").as_int();
	}else {// If invalid save default.
		RCLCPP_ERROR(this->get_logger(), "Invalid packet timer period. Setting to default: %d", DEFAULT_PACKET_TIMER_PERIOD);
		this->set_parameter(rclcpp::Parameter("packet_timer_period", DEFAULT_PACKET_TIMER_PERIOD));
		packet_timer_period_ = DEFAULT_PACKET_TIMER_PERIOD;
	}

	// IP Address - Read only
	rcl_interfaces::msg::ParameterDescriptor ip_address_description = rcl_interfaces::msg::ParameterDescriptor();
	ss << 	"IPv4 address to connect to the device on.\n"<<
			"  Default: 0.0.0.0";
	ip_address_description.description = ss.str();
	ss.str(""); // empty the stream
	ip_address_description.name = "ip_address";
	ip_address_description.read_only = true;
	this->declare_parameter<std::string>("ip_address", DEFAULT_IP_ADDRESS, ip_address_description);
	comms_data_.ip_address = this->get_parameter("ip_address").as_string();

	// Port - Read only
	rcl_interfaces::msg::ParameterDescriptor ip_port_description = rcl_interfaces::msg::ParameterDescriptor();
	ss << 	"Port to connect to the Advanced Navigation device on\n"<<
			"  Default: 0";
	ip_port_description.description = ss.str();
	ss.str(""); // empty the stream
	ip_port_description.name = "port";
	ip_port_description.read_only = true;
	rcl_interfaces::msg::IntegerRange portRange;
	portRange.from_value = MIN_PORT;
	portRange.to_value   = MAX_PORT;
	portRange.step 		 = 1;
	ip_port_description.integer_range.push_back(portRange);
	this->declare_parameter<int>("port", 0, ip_port_description);
	comms_data_.port = (int) this->get_parameter("port").as_int();

	// Logging Path - Read Only
	rcl_interfaces::msg::ParameterDescriptor log_path_description = rcl_interfaces::msg::ParameterDescriptor();
	ss << "Path for all logfiles to be placed. Default '~/.ros/log/'\n";
	log_path_description.description = ss.str();
	ss.str("");
	log_path_description.name = "log_path";
	log_path_description.read_only = true;
	this->declare_parameter<std::string>("log_path", "~/.ros/log/", log_path_description);
	log_path_ = this->get_parameter("log_path").as_string();


	// Comms Select - Read only
	rcl_interfaces::msg::ParameterDescriptor comms_select_description = rcl_interfaces::msg::ParameterDescriptor();
	ss << 	"What method will be used to communicate with the device.\n"<<
			"  Default: 0: Serial\n" <<
			"  1: TCP Client\n" <<
			"  2: TCP Server\n" <<
			"  3: UDP\n" <<
			"  4: CAN\n";
	comms_select_description.description = ss.str();
	ss.str(""); // empty the stream
	comms_select_description.name = "comm_select";
	comms_select_description.read_only = true;
	rcl_interfaces::msg::IntegerRange commRange;
	commRange.from_value = 0;
	commRange.to_value   = 4;
	commRange.step 		 = 1;
	comms_select_description.integer_range.push_back(commRange);
	this->declare_parameter<int>("comm_select", adnav::CONNECTION_SERIAL, comms_select_description);
	comms_data_.method = (int) this->get_parameter("comm_select").as_int();
}

//~~~~~~ Control Functions

/**
 * @brief Function to receive ANPP Packets from the COMPORT as specified in user input.
 *
 * Uses the ANPP SDK to decode the packets and call the relevant ROS decoder.
 *
 * This Function will also create a log file session and log incoming data to it.
 */
void Driver::recievePackets() {
	// initialize the decoder.
	an_decoder_t an_decoder;
	an_decoder_initialise(&an_decoder);
	int bytes_received;

	// get the bytes from the communication method, and load them into the decoder
	bytes_received = communicator_->read(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder));

	decodePackets(an_decoder, bytes_received);
}

/**
 * @brief Periodic callback for publishing ROS messages.
 *
 * This function accesses in a thread safe manner the class stored ROS messages, and publishes them to their respective topics.
 * Awaits a signal from a ROS Decoder method before publishing and accessing shared data.
 */
void Driver::publishTimerCallback() {
	// Since the messages can be filled in other threads ensure exclusive access.
	std::unique_lock<std::mutex> lock(messages_mutex_);

	// Debug timekeeper value
	rcl_time_point_value_t time = 0, diff = 0;

	// Check for updates from other threads.
	if(!msg_write_done_) {
		time = this->get_clock().get()->now().nanoseconds();
		// Only wait until timeout. otherwise log error and exit callback.
		if (msg_cv_.wait_for(lock, std::chrono::milliseconds(DEFAULT_TIMEOUT)) == std::cv_status::timeout) {
			RCLCPP_DEBUG(this->get_logger(), "Publish Timeout");
			if(time) diff = this->get_clock().get()->now().nanoseconds() - time;
			RCLCPP_DEBUG(this->get_logger(), "PubTimeout:\tAccess: %d\tTimeWait: %ld μs", pub_num_, diff/1000);
			return;
		}
	}

	// Debug message to show how long it waited to be awoken.
	if(time) diff = this->get_clock().get()->now().nanoseconds() - time;
	RCLCPP_DEBUG(this->get_logger(), "Pub: \t\tMutex: L\tAccess: %d\tTimeWait: %ld μs", pub_num_, diff/1000);

	// PUBLISH MESSAGES
	nav_sat_fix_pub_->publish(nav_fix_msg_);
	twist_pub_->publish(twist_msg_);
	imu_pub_->publish(imu_msg_);
	imu_raw_pub_->publish(imu_raw_msg_);
	system_status_pub_->publish(system_status_msg_);
	filter_status_pub_->publish(filter_status_msg_);
	magnetic_field_pub_->publish(mag_field_msg_);
	barometric_pressure_pub_->publish(baro_msg_);
	temperature_pub_->publish(temp_msg_);
	pose_pub_->publish(pose_msg_);

	// ADV-153 Phase 3: lightweight per-packet raw topics
	position_std_dev_pub_->publish(position_std_dev_msg_);
	velocity_std_dev_pub_->publish(velocity_std_dev_msg_);
	euler_std_dev_pub_->publish(euler_std_dev_msg_);
	ned_velocity_pub_->publish(ned_velocity_msg_);
	body_acceleration_pub_->publish(body_acceleration_msg_);
	euler_orientation_pub_->publish(euler_orientation_msg_);
	quaternion_orientation_pub_->publish(quaternion_orientation_msg_);
	angular_velocity_pub_->publish(angular_velocity_msg_);
	angular_acceleration_pub_->publish(angular_acceleration_msg_);

	RCLCPP_DEBUG(this->get_logger(), "Pub: \t\tMutex: U\tAccess: %d", pub_num_++);

	// Restore the blocking flag before exiting the lock guard.
	msg_write_done_ = false;
}

/**
 * @brief Function to cancel and restart the Publisher timer.
 */
void Driver::RestartPublisher() {
	RCLCPP_INFO(this->get_logger(), "Restart Publisher Timer");

	// Cancel old timer to stop callbacks from triggering while remaking timer.
	publish_timer_->cancel();

	// Reset the timer using the class stored interval.
	publish_timer_ = this->create_wall_timer(
      	publish_timer_interval_ , std::bind(&Driver::publishTimerCallback, this), publishing_group_);
}

/**
 * @brief Function to cancel and restart the Reader timer.
 */
void Driver::RestartReader() {
	RCLCPP_INFO(this->get_logger(), "Restarting Reader Timer");

	// Cancel old timer to stop callbacks from triggering while remaking timer.
	read_timer_->cancel();

	// Reset the timer using the class stored interval.
	read_timer_ = this->create_wall_timer(
		read_timer_interval_, std::bind(&Driver::recievePackets, this), reading_group_);
}

//~~~~~~ Logging Functions

/**
 * @brief Function to log an error message into the ROS system status and to the Error Logger
 *
 * This function assumes it is being called from a function with thread safe access to the ROS messages.
 *
 * @param errmsg string containing the message to be put into the logger.
 */
void Driver::statusErrLog(const std::string& errmsg) {

	system_status_msg_.level = 2; // ERROR state
	system_status_msg_.message += errmsg;
	RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000, errmsg.c_str());
}

/**
 * @brief Function to log an warning message into the ROS filter status and to the warning Logger
 *
 * This function assumes it is being called from a function with thread safe access to the ROS messages.
 *
 * @param warnmsg string containing the message to be put into the logger.
 */
void Driver::statusWarnLog(const std::string& warnmsg) {

	filter_status_msg_.level = diagnostic_msgs::msg::DiagnosticStatus::WARN; // WARN state
	filter_status_msg_.message += warnmsg;
	RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, warnmsg.c_str());
}

//~~~~~~ Service Functions

/**
 * @brief Service to update the Packet Periods from the device.
 *
 * Communicates to the device and receives an acknowledgement packet.
 *
 * @param request Const shared pointer to a PacketPeriods request.
 * @param response Shared pointer to a PacketPeriods response.
 */
void Driver::srvPacketPeriods(const std::shared_ptr<adnav_interfaces::srv::PacketPeriods::Request> request,
		std::shared_ptr<adnav_interfaces::srv::PacketPeriods::Response> response) {

	// Send Config to device
	SendPacketPeriods(request->periods, request->clear_existing_periods);

	// Await the response
	response->acknowledgement = AcknowledgeHandler();

	// notify of result.
	char buf[15];
	snprintf(buf, sizeof(buf)/sizeof(buf[0]), "%s%d%s", "Failure: ", response->acknowledgement.result, "\n");
	RCLCPP_INFO(this->get_logger(), "Received Update acknowledgement:\nID: %d\tCRC: %d\nOutcome: %s", response->acknowledgement.id,
            response->acknowledgement.crc, (response->acknowledgement.result == 0) ? "Success\n":buf);
}

/**
 * @brief Service to request a change of Packet Timer Period from the device.
 *
 * @param request Const shared pointer to a PacketTimerPeriod Request msg
 * @param response Shared pointer to a PacketTimerPeriod Response.
 */
void Driver::srvPacketTimerPeriod(const std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Request> request,
		std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Response> response) {

	// Send config to device.
	SendPacketTimer(request->packet_timer_period, request->utc_synchronisation, request->permanent);

	// Await the response.
	response->acknowledgement = AcknowledgeHandler();

	// notify of result.
	char buf[15];
	snprintf(buf, sizeof(buf)/sizeof(buf[0]), "%s%d%s", "Failure: ", response->acknowledgement.result, "\n");
	RCLCPP_INFO(this->get_logger(), "Received Update acknowledgement:\nID: %d\tCRC: %d\nOutcome: %s", response->acknowledgement.id,
            response->acknowledgement.crc, (response->acknowledgement.result == 0) ? "Success\n":buf);
}

/**
 * @brief Service to request a series of packets from the device.
 *
 * @param request Const shared pointer to a RequestPackets Request msg
 * @param response Shared pointer to a RequestPackets Response.
 */
void Driver::srvRequestPackets(const std::shared_ptr<adnav_interfaces::srv::RequestPackets::Request> request,
		std::shared_ptr<adnav_interfaces::srv::RequestPackets::Response> response) {

	an_packet_t *an_packet;

	RCLCPP_INFO(this->get_logger(), "Incoming Packet Request:");

	for(auto& i : request->packet_ids) {
		RCLCPP_INFO(this->get_logger(), "Requesting ID: %d", i);
		an_packet = encode_request_packet(i);
		encodeAndSend(an_packet);
	}

	response->success = true;
}

void Driver::srvNtrip(const std::shared_ptr<adnav_interfaces::srv::Ntrip::Request> request,
		std::shared_ptr<adnav_interfaces::srv::Ntrip::Response> response) {
	std::stringstream dbg_ss;
	dbg_ss << "NTRIP Service Called:\n\tEnable: " << request->enable << "\n\tUsername: " << request->username <<
		"\n\tPassword: " << request->password << "\n\tHost: " << request->host << "\n\tMountpoint: " << request->mountpoint << "\n";
	RCLCPP_DEBUG(this->get_logger(), "%s", dbg_ss.str().c_str());

	// Reset the state
	ntrip_state_ = {};

	// Fill out data from request
	ntrip_state_.en = request->enable;

	if (ntrip_state_.en) {
		ntrip_state_.username = request->username;
		ntrip_state_.password = request->password;
		ntrip_state_.mountpoint = request->mountpoint;
		// Decode the host str into IP and Port.
		getDataFromHostStr(request->host);

	} else {
		if(ntrip_client_.get() != nullptr && ntrip_client_->service_running()) ntrip_client_->stop();
		if(rtcm_logger_.is_open()) rtcm_logger_.closeFile();
		response->reason = std::string("Successfully disabled NTRIP Client.");
		response->success = true;
		return;
	}

	// Update the client
	updateNTRIPClientService();

	// If no mountpoint has been specified print out the sourcetable.
	if (ntrip_state_.mountpoint.empty()) {
		if (ntrip_client_->retrieve_sourcetable()) {
			response->success = true;
			response->reason = std::string("Successfully Requested Sourcetable");
		} else {
			response->success = false;
			switch (ntrip_client_->service_failure())
			 {
			case adnav::ntrip::NTRIP_CONNECTION_TIMEOUT_FAILURE:
				response->reason = std::string("NTRIP Response Timeout.");
				break;
			case adnav::ntrip::NTRIP_UNRECOGNIZED_RETURN:
				response->reason = std::string("Unrecognized Sourcetable Header");
				break;
			default:
				response->reason = std::string("Sourcetable Retieval Failure.");
				break;
			}
		}
		return;
	}

	// Start the service
	if(!ntrip_client_->run()) {
		ntrip_client_->stop();
		rtcm_logger_.closeFile();
	}

	// If the client is initialized
	if (ntrip_client_.get() != nullptr) {
		int attempts = 0;

		// While the ntrip service is not what is requested and hasn't failed
		while(ntrip_client_->service_running() != ntrip_state_.en &&
				ntrip_client_->service_failure() == adnav::ntrip::NTRIP_NO_FAIL) {
			attempts++;
			std::this_thread::sleep_for(std::chrono::seconds(1));
			if(attempts > NTRIP_TIMEOUT_PERIOD + 1) {
				response->success = false;
				response->reason = std::string("NTRIP Client Failure");
				if(ntrip_state_.he == nullptr) response->reason.append(": Bad Host");
				return;
			}
		}

		// if the service fails.
		if(ntrip_client_->service_failure() != adnav::ntrip::NTRIP_NO_FAIL) {
			response->success = false;
			std::stringstream ss;
			switch (ntrip_client_->service_failure())
			 {
			case adnav::ntrip::NTRIP_CONNECTION_TIMEOUT_FAILURE:
				ss << "NTRIP Connection Timeout.\n";
				break;
			case adnav::ntrip::NTRIP_CREATE_SOCK_FAILURE:
				ss << "NTRIP Socket Creation Failure.\n";
				break;
			case adnav::ntrip::NTRIP_REMOTE_CLOSE:
				ss << "NTRIP Remote Socket Closed.\n";
				break;
			case adnav::ntrip::NTRIP_REMOTE_SOCKET_FAILURE:
				ss << "NTRIP Remote Socket Failure.\n";
				break;
			case adnav::ntrip::NTRIP_CASTER_CONNECTION_FAILURE:
				ss << "NTRIP Caster Connection Failure.\n";
				if(ntrip_state_.he == nullptr) ss << ":Bad Host";
				break;
			case adnav::ntrip::NTRIP_SEND_GPGGA_FAILURE:
				ss << "NTRIP SEND GPGGA FAILURE.\n";
				break;
			case adnav::ntrip::NTRIP_UNAUTHORIZED:
				ss << "NTRIP Caster Access Authorisation Failure.\n";
				break;
			case adnav::ntrip::NTRIP_FORBIDDEN:
				ss << "NTRIP Caster returned HTTP 403 Forbidden.\n";
				break;
			case adnav::ntrip::NTRIP_NOT_FOUND:
				ss << "NTRIP Caster returned HTTP 404 Not Found. Perhaps a bad mountpoint or host was supplied?\n";
				break;
			case adnav::ntrip::NTRIP_UNRECOGNIZED_RETURN:
				ss << "NTRIP Caster returned an unknown HTTP header.\n";
				break;
			default:
				ss << "Unknown NTRIP Error.\n";
				break;
			}
			response->reason = ss.str();
			RCLCPP_ERROR(this->get_logger(), "%s", ss.str().c_str());
			rtcm_logger_.closeFile();
			return;
		}
	}

	// it is uninitialized and request is to enable ntrip
	else if (ntrip_state_.en == true) {
		RCLCPP_ERROR(this->get_logger(), "Unable to enable NTRIP service");
		response->success = false;
		response->reason = std::string("Unable to enable NTRIP service");
		return;
	}

	// Successful result reached
	response->success = true;
	if(ntrip_state_.en == true) response->reason = std::string("Successfuly enabled NTRIP client.");
	else response->reason = std::string("Successfully disabled NTRIP client.");

}

//~~~~~~ Parameter Functions

/**
 * @brief Callback to handle dynamic reconfiguration of node parameters.
 *
 * @param Params Vector of changed parameters
 * @return Result class for success/failure of parameter settings
 */
rcl_interfaces::msg::SetParametersResult Driver::ParamSetCallback(const std::vector<rclcpp::Parameter> &Params) {
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "";

	// For every parameter in the vector
	for (const auto &parameter : Params) {

		RCLCPP_INFO(this->get_logger(), "Received Param Update Request\n\tname:\t%s\n\ttype:\t%s\n\tvalue:\t%s\n", parameter.get_name().c_str(),
					parameter.get_type_name().c_str(), parameter.value_to_string().c_str());

		// Validation of parameters
		// Only if they require validation
		// Baud rate
		if (parameter.get_name() == "baud_rate") result.reason += validateBaudRate(parameter).reason;
		// comport
		else if (parameter.get_name() == "com_port") result.reason += validateComPort(parameter).reason;
		// Publisher Interval Duration
		else if (parameter.get_name() == "publish_us") result.reason += validatePublishUs(parameter).reason;
		// Sensor Polling Interval Duration
		else if (parameter.get_name() == "read_us") result.reason += validateReadUs(parameter).reason;
		// Packet IDs and Rates
		else if (parameter.get_name() == "packet_request") result.reason += validatePacketRequest(parameter).reason;
		// Packet Timer Period
		else if (parameter.get_name() == "packet_timer_period") result.reason += validatePacketTimer(parameter).reason;
	} // for each parameter

	if(result.reason != "") {
		result.successful = false;
	}
	return result;
}

/**
 * @brief Function to validate the proposed parameter change to the Baud Rate parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Baud Rate parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validateBaudRate(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of Integer Type.\n";
	}

	// test for valid baudrate
	switch(parameter.as_int())
	 {
		case    2400 : 	break;
		case    4800 : 	break;
		case    9600 : 	break;
		case   19200 : 	break;
		case   38400 : 	break;
		case   57600 : 	break;
		case  115200 : 	break;
		case  230400 : 	break;
		case  460800 : 	break;
		case  500000 : 	break;
		case  576000 : 	break;
		case  921600 : 	break;
		case 1000000 : 	break;
		case 2000000 : 	break;
		default      : 	result.successful = false;
						ss << "\n[Error] Invalid Baud Rate\n";
					   	break;
	}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

/**
 * @brief Function to validate the proposed parameter change to the Com Port parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Com Port parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validateComPort(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of String Type.\n";
	}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

/**
 * @brief Function to validate the proposed parameter change to the Publish us parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Publish us parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validatePublishUs(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of Integer Type.\n";
	}

	// Guard for less than read_us
	if(parameter.as_int() < this->get_parameter("read_us").as_int()) {
		result.successful = false;
		ss << "\n[Error] Publish period must be greater than or equal to read period\n";
	}

	// Guard for timer range
	if(parameter.as_int() < 1000 || parameter.as_int() > 1000000) {
		result.successful = false;
		ss << "\n[Error] Publish Period " << parameter.as_int() << " is invalid. Out of range.";
		result.reason = ss.str();
	}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

/**
 * @brief Function to Update the Publish us parameter, create temporary service client and update the node using services.
 *
 * @param parameter const rclcpp::Parameter. updated Publish us parameter
 */
void Driver::updatePublishUs(const rclcpp::Parameter& parameter) {
	publish_timer_interval_ = std::chrono::microseconds(parameter.as_int());

	RestartPublisher();
}

/**
 * @brief Function to validate the proposed parameter change to the Read us parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Read us parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validateReadUs(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of Integer Type.\n";
	}

	// Guard for timer range
	if(parameter.as_int() < 1000 || parameter.as_int() > 1000000) {
		result.successful = false;
		ss << "\n[Error] Read Period " << parameter.as_int() << " is invalid. Out of range.";
	}

	// Ensure that the publish timer is not lower than the new read period.
	try
	 {
		if(parameter.as_int() > this->get_parameter("publish_us").as_int()) {
			result.successful = false;
			ss << "[Error] Publish Period (" << this->get_parameter("publish_us").as_int() <<
				") is less than requested read period (" << parameter.as_int() << ").\n";
		}
	} // This catch will be thrown on startup as read is initialized period to publish.
	catch(const rclcpp::exceptions::ParameterNotDeclaredException& e) {}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

/**
 * @brief Function to Update the Read us parameter, create temporary service client and update node timer using service
 *
 * @param parameter const rclcpp::Parameter. updated Read us parameter
 */
void Driver::updateReadUs(const rclcpp::Parameter& parameter) {
	read_timer_interval_ = std::chrono::microseconds(parameter.as_int());

	RestartReader();
}

/**
 * @brief Function to validate the proposed parameter change to the Packet Request parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Packet Request parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validatePacketRequest(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of Integer Array Type.\n";
	}

	// Guard to ensure that a packet is included in the array
	if(parameter.as_integer_array().size() < 2) {
		result.successful = false;
		ss << "\n[Error] ID Array must contain one or more requested packet\n" <<
			"Usage: [ID1, Rate1, ID2, Rate2, ...]\n";
	}

	// Guard to ensure that there is even entries
	if(parameter.as_integer_array().size() % 2 != 0) {
		result.successful = false;
		ss << "\n[Error] ID Array must contain an even number of entries\n"<<
			"Usage: [ID1, Rate1, ID2, Rate2, ...]\n";
	}

	// Check all elements of the request array
	for(unsigned int i = 0; i < parameter.as_integer_array().size(); i++) {
		// if even (ID)
		int64_t element = parameter.as_integer_array().at(i);
		if(i%2 == 0) {
			// Check for valid ID
			if( element < 0 || 	// below range
			(element >= end_system_packets && element < START_STATE_PACKETS) || // system-state gap
			(element >= end_state_packets && element < START_CONFIGURATION_PACKETS) || // state-config gap
			element >= end_configuration_packets) { // above range
				result.successful = false;
				ss << "\n[Error] ID: " << element << "\t isn't a valid packet.\n";
			}
		}else { // If odd (Period)
			if(element < MIN_PACKET_PERIOD || element > MAX_PACKET_PERIOD) {
				result.successful = false;
				ss << "\n[Error] Period: " << element << "\t isn't a valid period.\n";
			}
		}
	}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

/**
 * @brief Function to Update the Packet Request parameter, create temporary service client and update the device using service
 *
 * @param parameter const rclcpp::Parameter. updated Packet Request parameter
 */
void Driver::updatePacketRequest(const rclcpp::Parameter& parameter) {

	packet_request_ = parameter.as_integer_array();

	// Create and fill a periods format.
	std::vector<adnav_interfaces::msg::PacketPeriod> packet_periods;
	for(uint64_t i = 0; (i+i) < packet_request_.size(); i++) {
		adnav_interfaces::msg::PacketPeriod period;
		period.packet_id = packet_request_[i+i];
		period.packet_period = packet_request_[i+i+1];
		packet_periods.push_back(period);
	}

	(void) SendPacketPeriods(packet_periods);
}

/**
 * @brief Function to validate the proposed parameter change to the Packet Timer parameter.
 *
 * @param parameter const rclcpp::Parameter. proposed change to the Packet Timer parameter.
 * @return returns a SetParametersResult message containing a success value and a string descriptor.
 */
rcl_interfaces::msg::SetParametersResult Driver::validatePacketTimer(const rclcpp::Parameter& parameter) {
	rcl_interfaces::msg::SetParametersResult result;
	result.reason = "";
	result.successful = true;

	// Stringstream to stream in faults.
	std::stringstream ss;

	// Guard for parameter type
	if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
		result.successful = false;
		ss << "\n[Error] Passed parameter is not of Integer Type.\n";
	}

	// Guard for timer range
	if(parameter.as_int() < MIN_TIMER_PERIOD || parameter.as_int() > MAX_TIMER_PERIOD) {
		result.successful = false;
		ss << "\n[Error] Packet Timer Period " << parameter.as_int() << " is invalid.";
		result.reason = ss.str();
	}

	if(!result.successful)	 {
		result.reason = ss.str();
	}
	return result;
}

/**
 * @brief Function to Update the PacketTimer parameter, then call for it to be sent to the device.
 *
 * @param parameter const rclcpp::Parameter. updated PacketTimer parameter
 */
void Driver::updatePacketTimer(const rclcpp::Parameter& parameter) {
	packet_timer_period_ = (int) parameter.as_int();

	// Send to the device.
	(void) SendPacketTimer(packet_timer_period_);
}

//~~~~~~ NTRIP Functions

/**
 * @brief Function to Update the NTRIP service with new parameters.
 */
void Driver::updateNTRIPClientService() {
	// check if the service is running, if so stop it
	if (ntrip_client_.get() != nullptr && ntrip_client_->service_running()) {
		ntrip_client_->stop();
	}

	if (rtcm_logger_.is_open()) rtcm_logger_.closeFile();

	// Guard to stop execution if request enable is false.
	if (ntrip_state_.en == false) return;

	// create new client instance with new parameters. Deletes old instance.
	ntrip_client_ = std::make_unique<adnav::ntrip::Client>(ntrip_state_.ip,
		ntrip_state_.port,
		ntrip_state_.username,
		ntrip_state_.password,
		ntrip_state_.mountpoint,
		[this](const std::string& msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); },
		[this](const std::string& msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); },
		"NTRIP AdNavRos/2.0");

	ntrip_client_->OnReceived(std::bind(
			&Driver::NtripReceiveFunction,
			this,
			std::placeholders::_1,
			std::placeholders::_2));

	ntrip_client_->set_report_interval(DEFAULT_GPGGA_REPORT_PERIOD);
	ntrip_client_->set_location(llh_.latitude, llh_.longitude, llh_.height);

	// Guard to stop bad hostend return.
	if(ntrip_state_.he != nullptr) {
		rtcm_logger_.openFile((std::string("Log_") + std::string(ntrip_state_.he->h_name)), ".rtcm", log_path_);
	}
}

/**
 * @brief Function to decode the host string and place relevant data into struct.
 *
 * @param host string containing host or ip followed by port
 */
void Driver::getDataFromHostStr(const std::string& host) {
	// split the string using the ':' character
	std::vector<std::string> split_string = adnav::utils::splitStr(host, ':');

	// if the string is invalid.
	if(split_string.size() != 2) {
		RCLCPP_ERROR(this->get_logger(), "Invalid Host String");
		this->set_parameter(rclcpp::Parameter("ntrip_enable", false));
	}

	struct in_addr addr;
	// Get the IP from the string.
	if(adnav::utils::validateIP(split_string.front()) == true) {
		if(inet_pton(AF_INET, split_string.front().c_str(), &addr) == 0) {
			RCLCPP_ERROR(this->get_logger(), "Invalid Address.");
			return;
		}
		ntrip_state_.he = gethostbyaddr((char*) &addr, sizeof(addr), AF_INET);
		if(ntrip_state_.he == nullptr) {
			ntrip_state_.he = gethostbyname(split_string.front().c_str());
		}
	} else {
		ntrip_state_.he = gethostbyname(split_string.front().c_str());
	}

	if(ntrip_state_.he == nullptr) return;

	char ip[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, (struct in_addr*) ntrip_state_.he->h_addr_list[0], ip, INET_ADDRSTRLEN);

	ntrip_state_.ip = std::string(ip);


	// Get the Port from the string
	ntrip_state_.port = atoi(split_string.back().c_str());

	return;
}

/**
 * @brief Function to take the incoming buffer from the NTRIP client, place it into ANPP Packet 55
 * (RTCM Corrections Packet) and send it to the device.
 *
 * @param buffer the character buffer containing RTCM data.
 * @param size size of the RTCM corrections buffer.
 */
void Driver::NtripReceiveFunction(const char* buffer, int size) {
	std::stringstream ss;
	an_packet_t* an_packet;
	rtcm_corrections_packet_t rtcm_corrections_packet;
	std::string received(buffer, size);

	// Write to the logfile.
	rtcm_logger_.writeAndIncrement(buffer, size);

	ss << "Recieved [" << size << "]:\n";
	// for each byte in the buffer
	int buffer_idx = 0;
	int number_of_full_size_anpp = floor(size / 255);
	int size_of_last_packet = size % 255;
	for (int i = 0; i < number_of_full_size_anpp; ++i) {
		// Write to the debug string
		char buf[10];
		sprintf(buf, "%02X ", static_cast<uint8_t>(buffer[buffer_idx]));
		ss << buf;

		rtcm_corrections_packet.packet_data = (uint8_t*)(&buffer[buffer_idx]);
		// The packet is full send it to the device.
		an_packet = encode_rtcm_corrections_packet(&rtcm_corrections_packet, 255);
		RCLCPP_DEBUG(this->get_logger(), "Sending RTCM Corrections Packet with %d bytes", an_packet->length);
		encodeAndSend(an_packet);
		buffer_idx += 255;
	}

	// Send the final partially filled packet
	rtcm_corrections_packet.packet_data = (uint8_t*)(&buffer[buffer_idx]);
	an_packet = encode_rtcm_corrections_packet(&rtcm_corrections_packet, size_of_last_packet);
	RCLCPP_DEBUG(this->get_logger(), "Sending RTCM Corrections Packet with %d bytes", an_packet->length);
	encodeAndSend(an_packet);

	ss << std::endl << std::endl;
	RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
}

//~~~~~~ Device Communication Functions

/**
 * @brief Function to encode a ANPP packet with a header CRC and LRC, and send it over the open communications method
 *
 * @param an_packet a pointer to an an_packet_t object which will be encoded and send to the device
 */
void Driver::encodeAndSend(an_packet_t* an_packet) {
	an_packet_encode(an_packet);
	// Send encoded packet through the selected communication medium
	communicator_->write(an_packet_pointer(an_packet), an_packet_size(an_packet));
	// Free the packet to avoid memory leaks
	an_packet_free(&an_packet);
}

/**
 * @brief Function to handle the acknowledgement of configuration from a device in a thread safe manner.
 *
 * Needs to interact with the reading thread to receive the packet. Uses a conditional variable to flag when a
 * packet has been received.
 *
 * @return Raw Acknowledge Message
 */
adnav_interfaces::msg::RawAcknowledge Driver::AcknowledgeHandler() {
	// make acknowledgement data structure
	adnav_interfaces::msg::RawAcknowledge msg;

	// Wait for an acknowledge packet to be received.
	std::unique_lock<std::mutex> lock(acknowledge_mutex_);
	if(!acknowledge_recieve_) {
		if(srv_cv_.wait_for(lock, std::chrono::seconds(DEFAULT_TIMEOUT)) == std::cv_status::timeout) {
			RCLCPP_ERROR(this->get_logger(), "acknowledgement Timeout");
			msg.result++; // make error condition
			return msg;
		}
	}

	// acknowledge_packet_
	msg.id  = acknowledge_packet_.packet_id;
	msg.crc = acknowledge_packet_.packet_crc;
	msg.result = acknowledge_packet_.acknowledge_result;

	// reset condition variable.
	acknowledge_recieve_ = false;

	return msg;
}

/**
 * @brief Function to send the packet Timer to the device and await the acknowledgement.
 *
 * @param packet_timer_period Period for the packet rates in microseconds.
 * @param UTC_sync Synchronize with UTC time. True by default.
 * @param permanent Is this a permanent change. True by default.
 * @return acknowledgement Message
 */
adnav_interfaces::msg::RawAcknowledge Driver::SendPacketTimer(int packet_timer_period, bool utc_sync, bool permanent) {
	RCLCPP_DEBUG_STREAM(this->get_logger(), "Incoming Packet Timer Request:\nUTC Sync: " <<
		(utc_sync ? "True\n":"False\n") <<
		"\tPermanent: " << (permanent ? "True\n":"False\n") <<
		"\tPeriod (μs): " << packet_timer_period << "\n");

	// Create a periods packet.
	packet_timer_period_packet_t packet_timer_period_packet;
	an_packet_t *an_packet;

	// set packet to 0
	memset(&packet_timer_period_packet, 0, sizeof(packet_timer_period_packet));

	// Fill the packet
	packet_timer_period_packet.permanent = permanent;
	packet_timer_period_packet.utc_synchronisation = utc_sync;
	packet_timer_period_packet.packet_timer_period = packet_timer_period;

	// Send the packet.
	RCLCPP_INFO(this->get_logger(), "Sending Packet Timer Request to device.");
	an_packet = encode_packet_timer_period_packet(&packet_timer_period_packet);
	encodeAndSend(an_packet);

	// Return the acknowledgement result.
	return AcknowledgeHandler();
}

/**
 * @brief Function to send the packet Timer to the device and await the acknowledgement.
 *
 * @param periods Vector of packet periods to be sent to the device.
 * @param clear_existing Boolean value for overwriting existing packet periods. Default = True.
 * @param permanent Boolean value for overwriting configuration memory. Default = True.
 * @return acknowledgement Message
 */
adnav_interfaces::msg::RawAcknowledge Driver::SendPacketPeriods(const std::vector<adnav_interfaces::msg::PacketPeriod>& periods,
	bool clear_existing, bool permanent) {
	std::stringstream ss;
	ss << "incoming Packet Request:\nClear: " << (clear_existing ? "True\n":"False\n") <<
		"Permanent: " << (permanent ? "True\n":"False\n") <<
		"Number Requested: " << periods.size() << std::endl;

	// Create a periods packet.
	packet_periods_packet_t packet_periods_packet;
	an_packet_t *an_packet;

	// Set contents of packet to 0
	memset(&packet_periods_packet, 0, sizeof(packet_periods_packet));

	// Fill in the packet
	packet_periods_packet.permanent = permanent;
	packet_periods_packet.clear_existing_packets = clear_existing;

	int j = 0;
	for(adnav_interfaces::msg::PacketPeriod i : periods) {
		ss << "\tID: " << (int)i.packet_id << "\tPeriod: "<< i.packet_period << std::endl;
		packet_periods_packet.packet_periods[j].packet_id = i.packet_id;
		packet_periods_packet.packet_periods[j].period = i.packet_period;
		j++;
	}

	RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());

	RCLCPP_INFO(this->get_logger(), "Sending Packet Periods Request to device.");
	an_packet = encode_packet_periods_packet(&packet_periods_packet);
	encodeAndSend(an_packet);

	return AcknowledgeHandler();
}

//~~~~~~ Decoders

/**
 * @brief Function to decode packets in the decoder and fill out ROS messages
 *
 * @param an_decoder instance of the decoder loaded with data to be decoded.
 * @param bytes number of bytes within the decoders buffer.
 */
void Driver::decodePackets(an_decoder_t &an_decoder, const int &bytes) {
	// If there are bytes to be decoded.
	an_packet_t* an_packet;
	if (bytes > 0) {

		anpp_logger_.writeAndIncrement(reinterpret_cast<char*>(an_decoder_pointer(&an_decoder)), bytes);
		// Increment the decode buffer length by the number of bytes received
		an_decoder_increment(&an_decoder, bytes);

		// Decode all the packets in the buffer
		while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
		 {
			RCLCPP_DEBUG(this->get_logger(), "ID: %d", an_packet->id);

			switch (an_packet->id)
			 {
			case packet_id_device_information: deviceInfoDecoder(an_packet);
				break;

			case packet_id_acknowledge: acknowledgeDecoder(an_packet);
				break;

			case packet_id_system_state: systemStateRosDecoder(an_packet);
				break;

			case packet_id_ecef_position: ecefPosRosDecoder(an_packet);
				break;

			case packet_id_quaternion_orientation_standard_deviation: quartOrientSDRosDriver(an_packet);
				break;

			case packet_id_raw_sensors: rawSensorsRosDecoder(an_packet);
				break;

			// ADV-153 Phase 3: lightweight per-packet raw topics
			case packet_id_status: statusRosDecoder(an_packet);
				break;

			case packet_id_position_standard_deviation: positionStdDevRosDecoder(an_packet);
				break;

			case packet_id_velocity_standard_deviation: velocityStdDevRosDecoder(an_packet);
				break;

			case packet_id_euler_orientation_standard_deviation: eulerStdDevRosDecoder(an_packet);
				break;

			case packet_id_ned_velocity: nedVelocityRosDecoder(an_packet);
				break;

			case packet_id_body_acceleration: bodyAccelRosDecoder(an_packet);
				break;

			case packet_id_euler_orientation: eulerOrientationRosDecoder(an_packet);
				break;

			case packet_id_quaternion_orientation: quaternionOrientRosDecoder(an_packet);
				break;

			case packet_id_angular_velocity: angularVelRosDecoder(an_packet);
				break;

			case packet_id_angular_acceleration: angularAccelRosDecoder(an_packet);
				break;

			default:
				RCLCPP_WARN/*_THROTTLE*/(this->get_logger(), /* *this->get_clock(), 500, */
					"Unsupported packet definition for ROS driver. PACKET_ID: %d", an_packet->id);
				break;
			}
			// Ensure that you free the an_packet when your done with it or you will leak memory
			an_packet_free(&an_packet);
		} // while an_packet != NULL
	}// if bytes > 0
}

/**
 * @brief Function to decode a acknowledgement packet and notify relevant services.
 *
 * @param an_packet pointer to a an_packet_t object from which to decode the information.
 */
void Driver::acknowledgeDecoder(an_packet_t* an_packet) {
	// worker thread gets lock
	std::unique_lock<std::mutex> lock(acknowledge_mutex_);

	// Decode packet and warn if error.
	if(decode_acknowledge_packet(&acknowledge_packet_, an_packet))
	 {
		RCLCPP_WARN(this->get_logger(), "Error decoding Acknowledge Packet");
	}

	RCLCPP_DEBUG(this->get_logger(), "acknowledgement received.\nID: %d\nResult: %d\n",
		acknowledge_packet_.packet_id, acknowledge_packet_.acknowledge_result);

	// Set the acknowledgement received to true
	acknowledge_recieve_ = true;

	// Notify the service thread
	srv_cv_.notify_one();
}

/**
 * @brief Function to decode a device information packet and output its contents to the ROS logger
 *
 * @param an_packet pointer to a an_packet_t object from which to decode the information.
 */
void Driver::deviceInfoDecoder(an_packet_t* an_packet) {
	// Decode packet and warn if error.
	if(decode_device_information_packet(&device_information_packet_, an_packet))
	 {
		RCLCPP_WARN(this->get_logger(), "Error decoding device information Packet");
	}
	// since multiple packets may be requested before the device responds. ensure only one gets printed per second.
	RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Device Information:\n"
			<< "Device ID: " <<	device_information_packet_.device_id <<
			"\nVersion:" <<
			"\n  Software: " << device_information_packet_.software_version <<
			"\n  Hardware: " << device_information_packet_.hardware_revision <<
			"\nSerial Number: " << std::hex << device_information_packet_.serial_number[0] <<
				device_information_packet_.serial_number[1] << device_information_packet_.serial_number[2]
			<< std::endl
			);

}

/**
 * @brief Function to decode the System State ANPP Packet (ANPP.20).
 *
 * This function accesses in a thread safe manner the class stored ROS messages, placed relevant information into them,
 * then using the publishing control variable, requests a publisher thread to publish the message.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::systemStateRosDecoder(an_packet_t* an_packet) {

	system_state_packet_t system_state_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);
	RCLCPP_DEBUG(this->get_logger(), "Packet 20:\tMutex: L\tAccess: %d", P20_num_);
	// Debug timekeeper
	auto time = this->get_clock().get()->now().nanoseconds();

	if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
	 {
			// NAVSATFIX
			nav_fix_msg_.header.stamp.sec = system_state_packet.unix_time_seconds;
			nav_fix_msg_.header.stamp.nanosec = system_state_packet.microseconds*1000;
			nav_fix_msg_.header.frame_id = frame_id_;
			if ((system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_2d) ||
				(system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_3d))
			 {
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
			}
			else if ((system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_sbas) ||
					(system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_omnistar))
			 {
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
			}
			else if ((system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_differential) ||
					(system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_rtk_float) ||
					(system_state_packet.filter_status.b.gnss_fix_type == gnss_fix_rtk_fixed))
			 {
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
			}
			else
			 {
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
				nav_fix_msg_.status.status= sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
			}
			nav_fix_msg_.latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
			nav_fix_msg_.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
			nav_fix_msg_.altitude = system_state_packet.height;
			nav_fix_msg_.position_covariance = { pow(system_state_packet.standard_deviation[1], 2), 0.0, 0.0,
				0.0, pow(system_state_packet.standard_deviation[0], 2), 0.0,
				0.0, 0.0, pow(system_state_packet.standard_deviation[2], 2)};
			nav_fix_msg_.position_covariance_type = nav_fix_msg_.COVARIANCE_TYPE_DIAGONAL_KNOWN;

			llh_.latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
			llh_.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
			llh_.height = system_state_packet.height;

			if(ntrip_client_.get() != nullptr) {
				ntrip_client_->set_location(llh_.latitude, llh_.longitude, llh_.height);
			}
			// TWIST (DEPRECATED - see twist_msg_ in adnav_driver.h; raw NED/FRD passthrough, no
			// frame conversion, not consumed by the new pipeline. Left as-is for compatibility.)
			twist_msg_.linear.x = system_state_packet.velocity[0];
			twist_msg_.linear.y = system_state_packet.velocity[1];
			twist_msg_.linear.z = system_state_packet.velocity[2];
			twist_msg_.angular.x = system_state_packet.angular_velocity[0];
			twist_msg_.angular.y = system_state_packet.angular_velocity[1];
			twist_msg_.angular.z = system_state_packet.angular_velocity[2];


			// ADV-153 Phase 3: imu_msg_.orientation/angular_velocity/linear_acceleration (and
			// pose_msg_.orientation) are no longer sourced from Packet 20 here - they now come
			// from the lightweight Packets 39 (EulerOrientation)/42 (AngularVelocity)/38
			// (BodyAcceleration) respectively, in eulerOrientationRosDecoder()/angularVelRosDecoder()/
			// bodyAccelRosDecoder() below, which also apply the FRD->FLU fix. imu_msg_.header.stamp
			// still comes from here since Packet 20 remains the only packet with a UTC timestamp
			// requested (kept at low rate for diagnostics/nav_sat_fix per the design doc).
			imu_msg_.header.stamp.sec = system_state_packet.unix_time_seconds;
			imu_msg_.header.stamp.nanosec = system_state_packet.microseconds*1000;
			imu_msg_.header.frame_id = frame_id_;

			// ADV-153 Phase 3: system_status_msg_/filter_status_msg_ are no longer built from
			// Packet 20 here - see statusRosDecoder() (Packet 23) below, which reuses this exact
			// logic against the lightweight status_packet_t instead of the monolithic
			// system_state_packet_t.
	}
	// Now that work is complete notify an update for the publisher.
	msg_write_done_ = true;
	msg_cv_.notify_one();
	auto diff = this->get_clock().get()->now().nanoseconds() - time;
	RCLCPP_DEBUG(this->get_logger(), "Packet 20:\tMutex: U\tAccess: %d\tTimeLocked: %ld μs", P20_num_++, diff/1000);
}

/**
 * @brief Function to decode the ECEF Position ANPP Packet (ANPP.33).
 *
 * This function accesses in a thread safe manner the class stored ROS messages, placed relevant information into them,
 * then using the publishing control variable, requests a publisher thread to publish the message.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::ecefPosRosDecoder(an_packet_t* an_packet) {
	ecef_position_packet_t ecef_position_packet;

	std::unique_lock<std::mutex> lock(messages_mutex_);
	RCLCPP_DEBUG(this->get_logger(), "Packet 33:\tMutex: L\tAccess: %d", P33_num_);
	// Debug timekeeper
	auto time = this->get_clock().get()->now().nanoseconds();

	// ECEF Position (in meters) Packet for Pose Message
	if(decode_ecef_position_packet(&ecef_position_packet, an_packet) == 0)
	 {
		pose_msg_.position.x = ecef_position_packet.position[0];
		pose_msg_.position.y = ecef_position_packet.position[1];
		pose_msg_.position.z = ecef_position_packet.position[2];
	}
	// Now that work is complete notify an update for the publisher.
	msg_write_done_ = true;
	msg_cv_.notify_one();
	// RCLCPP_DEBUG(this->get_logger(), "Raw: \tNotifying Complete\t%d", raw_num_++);
	auto diff = this->get_clock().get()->now().nanoseconds() - time;
	RCLCPP_DEBUG(this->get_logger(), "Packet 33:\tMutex: U\tAccess: %d\tTimeLocked: %ld μs", P33_num_++, diff/1000);
}

/**
 * @brief Function to decode the Quaternion Orientation Standard Deviation ANPP Packet (ANPP.27).
 *
 * This function accesses in a thread safe manner the class stored ROS messages, placed relevant information into them,
 * then using the publishing control variable, requests a publisher thread to publish the message.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::quartOrientSDRosDriver(an_packet_t* an_packet) {
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);
	RCLCPP_DEBUG(this->get_logger(), "Packet 27: \tMutex: L\tAccess: %d", P27_num_);
	// Debug timekeeper
	auto time = this->get_clock().get()->now().nanoseconds();

	if(decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0)
	 {
		// IMU message
		imu_msg_.orientation_covariance[0] = quaternion_orientation_standard_deviation_packet.standard_deviation[0];
		imu_msg_.orientation_covariance[4] = quaternion_orientation_standard_deviation_packet.standard_deviation[1];
		imu_msg_.orientation_covariance[8] = quaternion_orientation_standard_deviation_packet.standard_deviation[2];
	}
	// Now that work is complete notify an update for the publisher.
	msg_write_done_ = true;
	msg_cv_.notify_one();
	// RCLCPP_DEBUG(this->get_logger(), "Raw: \tNotifying Complete\t%d", raw_num_++);
	auto diff = this->get_clock().get()->now().nanoseconds() - time;
	RCLCPP_DEBUG(this->get_logger(), "Packet 27:\tMutex: U\tAccess: %d\tTimeLocked: %ld μs", P27_num_++, diff/1000);
}

/**
 * @brief Function to decode the Raw Sensor ANPP Packet (ANPP.28).
 *
 * This function accesses in a thread safe manner the class stored ROS messages, placed relevant information into them,
 * then using the publishing control variable, requests a publisher thread to publish the message.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::rawSensorsRosDecoder(an_packet_t* an_packet) {
	raw_sensors_packet_t raw_sensors_packet;

	std::unique_lock<std::mutex> lock(messages_mutex_);

	RCLCPP_DEBUG(this->get_logger(), "Packet 28: \tMutex: L\tAccess: %d", P28_num_);
	// Debug timekeeper
	auto time = this->get_clock().get()->now().nanoseconds();

	// Fill the messages
	if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0) {

		// RAW MAGNETICFIELD VALUE FROM IMU
		mag_field_msg_.header.frame_id = frame_id_;
		mag_field_msg_.magnetic_field.x = raw_sensors_packet.magnetometers[0];
		mag_field_msg_.magnetic_field.y = raw_sensors_packet.magnetometers[1];
		mag_field_msg_.magnetic_field.z = raw_sensors_packet.magnetometers[2];

		imu_raw_msg_.header.frame_id = frame_id_;
		imu_raw_msg_.orientation_covariance[0] = -1; // Tell recievers that no orientation is sent.
		// ADV-153: FRD->FLU fix (confirmed bug in S0 spike + live bench test: raw passthrough gave
		// linear_acceleration.z = -9.75 at rest instead of +9.8). Static 180deg rotation about X:
		// (x, -y, -z).
		imu_raw_msg_.linear_acceleration.x = raw_sensors_packet.accelerometers[0];
		imu_raw_msg_.linear_acceleration.y = -raw_sensors_packet.accelerometers[1];
		imu_raw_msg_.linear_acceleration.z = -raw_sensors_packet.accelerometers[2];
		imu_raw_msg_.angular_velocity.x = raw_sensors_packet.gyroscopes[0];
		imu_raw_msg_.angular_velocity.y = -raw_sensors_packet.gyroscopes[1];
		imu_raw_msg_.angular_velocity.z = -raw_sensors_packet.gyroscopes[2];

		// BAROMETRIC PRESSURE
		baro_msg_.header.frame_id = frame_id_;
		baro_msg_.fluid_pressure = raw_sensors_packet.pressure;

		// TEMPERATURE
		temp_msg_.header.frame_id = frame_id_;
		temp_msg_.temperature = raw_sensors_packet.pressure_temperature;

	}
	// Now that work is complete notify an update for the publisher.
	msg_write_done_ = true;
	msg_cv_.notify_one();
	// RCLCPP_DEBUG(this->get_logger(), "Raw: \tNotifying Complete\t%d", raw_num_++);

	auto diff = this->get_clock().get()->now().nanoseconds() - time;
	RCLCPP_DEBUG(this->get_logger(), "Packet 28:\tMutex: U\tAccess: %d\tTimeLock: %ld μs", P28_num_++, diff/1000);
}

// ADV-153 Phase 3: decoders for packets 23/24/25/26/35/38/39/40/42/43.

/**
 * @brief Function to decode the Status ANPP Packet (ANPP.23).
 *
 * Lightweight replacement for reading System Status / Filter Status off Packet 20 - same two
 * bitfields, without the position/velocity/orientation payload. Rebuilds system_status_msg_ /
 * filter_status_msg_ (the human-readable diagnostic_msgs/DiagnosticStatus topics) the same way
 * systemStateRosDecoder() used to, just sourced from status_packet_t instead of
 * system_state_packet_t.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::statusRosDecoder(an_packet_t* an_packet) {
	status_packet_t status_packet;
	std::stringstream ss;
	std::unique_lock<std::mutex> lock(messages_mutex_);
	auto time = this->get_clock().get()->now().nanoseconds();

	if (decode_status_packet(&status_packet, an_packet) == 0) {
		// SYSTEM STATUS
		system_status_msg_.message = "";
		system_status_msg_.level = diagnostic_msgs::msg::DiagnosticStatus::OK; // default OK state
		std::stringstream serial_num;
		serial_num << std::hex << device_information_packet_.serial_number[0] <<
			device_information_packet_.serial_number[1] << device_information_packet_.serial_number[2];
		system_status_msg_.hardware_id = serial_num.str();
		if (status_packet.system_status.b.system_failure) {
			ss << "\n0. SYSTEM FAILURE DETECTED.";
		}
		if (status_packet.system_status.b.accelerometer_sensor_failure) {
			ss << "\n1. ACCELEROMETER SENSOR FAILURE.";
		}
		if (status_packet.system_status.b.gyroscope_sensor_failure) {
			ss << "\n2. GYROSCOPE SENSOR FAILURE.";
		}
		if (status_packet.system_status.b.magnetometer_sensor_failure) {
			ss << "\n3. MAGNETOMETER SENSOR FAILURE.";
		}
		if (status_packet.system_status.b.pressure_sensor_failure) {
			ss << "\n4. PRESSURE SENSOR FAILURE.";
		}
		if (status_packet.system_status.b.gnss_failure) {
			ss << "\n5. GNSS FAILURE.";
		}
		if (status_packet.system_status.b.accelerometer_over_range) {
			ss << "\n6. ACCELEROMETER OVER RANGE.";
		}
		if (status_packet.system_status.b.gyroscope_over_range) {
			ss << "\n7. GYROSCOPE OVER RANGE.";
		}
		if (status_packet.system_status.b.magnetometer_over_range) {
			ss << "\n8. MAGNETOMETER OVER RANGE.";
		}
		if (status_packet.system_status.b.pressure_over_range) {
			ss << "\n9. PRESSURE OVER RANGE.";
		}
		if (status_packet.system_status.b.minimum_temperature_alarm) {
			ss << "\n10. MINIMUM TEMPERATURE ALARM.";
		}
		if (status_packet.system_status.b.maximum_temperature_alarm) {
			ss << "\n11. MAXIMUM TEMPERATURE ALARM.";
		}
		if (status_packet.system_status.b.internal_data_logging_error) {
			ss << "\n12. INTERNAL DATA LOGGING ERROR.";
		}
		if (status_packet.system_status.b.high_voltage_alarm) {
			ss << "\n13. HIGH VOLTAGE ALARM.";
		}
		// Note: this field is named gnss_antenna_disconnected in status_packet_t (packet 23) vs
		// gnss_antenna_fault in system_state_packet_t (packet 20) - same bit position (14), just
		// an inconsistent name in the vendored an-ros-common header.
		if (status_packet.system_status.b.gnss_antenna_disconnected) {
			ss << "\n14. GNSS ANTENNA FAULT.";
		}
		if (status_packet.system_status.b.serial_port_overflow_alarm) {
			ss << "\n15. SERIAL PORT DATA OVERFLOW.";
		}

		// If an error occurred log it
		if (!ss.str().empty()) {
			ss << std::endl;
			statusErrLog(ss.str());
			ss.str("");
		}

		// FILTER STATUS
		filter_status_msg_.message = "";
		filter_status_msg_.level = diagnostic_msgs::msg::DiagnosticStatus::OK; // default OK state
		filter_status_msg_.hardware_id = serial_num.str();
		if (status_packet.filter_status.b.orientation_filter_initialised) {
			filter_status_msg_.message += "\n0. Orientation Filter Initialised.";
		} else {
			ss << "\n0. Orientation Filter NOT Initialised.";
		}
		if (status_packet.filter_status.b.ins_filter_initialised) {
			filter_status_msg_.message += "\n1. Navigation Filter Initialised.";
		} else {
			ss << "\n1. Navigation Filter NOT Initialised.";
		}
		if (status_packet.filter_status.b.heading_initialised) {
			filter_status_msg_.message += "\n2. Heading Initialised.";
		} else {
			ss << "\n2. Heading NOT Initialised.";
		}
		if (status_packet.filter_status.b.utc_time_initialised) {
			filter_status_msg_.message += "\n3. UTC Time Initialised.";
		} else {
			ss << "\n3. UTC Time NOT Initialised.";
		}
		if (status_packet.filter_status.b.event1_flag) {
			ss << "\n7. Event 1 Occured.";
		} else {
			filter_status_msg_.message += "\n7. Event 1 NOT Occured.";
		}
		if (status_packet.filter_status.b.event2_flag) {
			ss << "\n8. Event 2 Occured.";
		} else {
			filter_status_msg_.message += "\n8. Event 2 NOT Occured.";
		}
		if (status_packet.filter_status.b.internal_gnss_enabled) {
			filter_status_msg_.message += "\n9. Internal GNSS Enabled.";
		} else {
			ss << "\n9. Internal GNSS NOT Enabled.";
		}
		if (status_packet.filter_status.b.dual_antenna_heading_active) {
			filter_status_msg_.message += "\n10. Dual Antenna Heading Active.";
		} else {
			ss << "\n10. Dual Antenna Heading NOT Active.";
		}
		if (status_packet.filter_status.b.velocity_heading_enabled) {
			filter_status_msg_.message += "\n11. Velocity Heading Enabled.";
		} else {
			ss << "\n11. Velocity Heading NOT Enabled.";
		}
		if (status_packet.filter_status.b.atmospheric_altitude_enabled) {
			filter_status_msg_.message += "\n12. Atmospheric Altitude Enabled.";
		} else {
			ss << "\n12. Atmospheric Altitude NOT Enabled.";
		}
		if (status_packet.filter_status.b.external_position_active) {
			filter_status_msg_.message += "\n13. External Position Active.";
		} else {
			ss << "\n13. External Position NOT Active.";
		}
		if (status_packet.filter_status.b.external_velocity_active) {
			filter_status_msg_.message += "\n14. External Velocity Active.";
		} else {
			ss << "\n14. External Velocity NOT Active.";
		}
		if (status_packet.filter_status.b.external_heading_active) {
			filter_status_msg_.message += "\n15. External Heading Active.";
		} else {
			ss << "\n16. External Heading NOT Active.";
		}

		// If a warning has occured log it
		if (!ss.str().empty()) {
			ss << std::endl;
			statusWarnLog(ss.str());
			ss.str("");
		}
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
	auto diff = this->get_clock().get()->now().nanoseconds() - time;
	RCLCPP_DEBUG(this->get_logger(), "Packet 23:\tMutex: U\tTimeLocked: %ld μs", diff/1000);
}

/**
 * @brief Function to decode the Position Standard Deviation ANPP Packet (ANPP.24).
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::positionStdDevRosDecoder(an_packet_t* an_packet) {
	position_standard_deviation_packet_t position_standard_deviation_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);

	if (decode_position_standard_deviation_packet(&position_standard_deviation_packet, an_packet) == 0) {
		position_std_dev_msg_.header.stamp = this->get_clock()->now();
		position_std_dev_msg_.header.frame_id = frame_id_;
		position_std_dev_msg_.anpp_header.header_lrc = an_packet->header[0];
		position_std_dev_msg_.anpp_header.id = an_packet->id;
		position_std_dev_msg_.anpp_header.length = an_packet->length;
		position_std_dev_msg_.anpp_header.crc = an_packet->header[3] | (an_packet->header[4] << 8);

		position_std_dev_msg_.standard_deviation.latitude = position_standard_deviation_packet.standard_deviation[0];
		position_std_dev_msg_.standard_deviation.longitude = position_standard_deviation_packet.standard_deviation[1];
		position_std_dev_msg_.standard_deviation.height = position_standard_deviation_packet.standard_deviation[2];
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
}

/**
 * @brief Function to decode the Velocity Standard Deviation ANPP Packet (ANPP.25).
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::velocityStdDevRosDecoder(an_packet_t* an_packet) {
	velocity_standard_deviation_packet_t velocity_standard_deviation_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);

	if (decode_velocity_standard_deviation_packet(&velocity_standard_deviation_packet, an_packet) == 0) {
		velocity_std_dev_msg_.header.stamp = this->get_clock()->now();
		velocity_std_dev_msg_.header.frame_id = frame_id_;
		velocity_std_dev_msg_.anpp_header.header_lrc = an_packet->header[0];
		velocity_std_dev_msg_.anpp_header.id = an_packet->id;
		velocity_std_dev_msg_.anpp_header.length = an_packet->length;
		velocity_std_dev_msg_.anpp_header.crc = an_packet->header[3] | (an_packet->header[4] << 8);

		// Native NED nav frame - intentionally not rotated here, see NedVelocity.msg / the design doc.
		velocity_std_dev_msg_.standard_deviation.north = velocity_standard_deviation_packet.standard_deviation[0];
		velocity_std_dev_msg_.standard_deviation.east = velocity_standard_deviation_packet.standard_deviation[1];
		velocity_std_dev_msg_.standard_deviation.down = velocity_standard_deviation_packet.standard_deviation[2];
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
}

/**
 * @brief Function to decode the Euler Orientation Standard Deviation ANPP Packet (ANPP.26).
 *
 * Also wires the roll/pitch/heading variance directly into imu_msg_.orientation_covariance's
 * diagonal (indices 0/4/8 of the row-major 3x3 matrix) - same frame, no rotation needed since the
 * NED->ENU yaw sign flip applied to the mean heading does not change its variance.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::eulerStdDevRosDecoder(an_packet_t* an_packet) {
	euler_orientation_standard_deviation_packet_t euler_orientation_standard_deviation_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);

	if (decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0) {
		euler_std_dev_msg_.header.stamp = this->get_clock()->now();
		euler_std_dev_msg_.header.frame_id = frame_id_;
		euler_std_dev_msg_.anpp_header.header_lrc = an_packet->header[0];
		euler_std_dev_msg_.anpp_header.id = an_packet->id;
		euler_std_dev_msg_.anpp_header.length = an_packet->length;
		euler_std_dev_msg_.anpp_header.crc = an_packet->header[3] | (an_packet->header[4] << 8);

		euler_std_dev_msg_.standard_deviation.roll = euler_orientation_standard_deviation_packet.standard_deviation[0];
		euler_std_dev_msg_.standard_deviation.pitch = euler_orientation_standard_deviation_packet.standard_deviation[1];
		euler_std_dev_msg_.standard_deviation.heading = euler_orientation_standard_deviation_packet.standard_deviation[2];

		imu_msg_.orientation_covariance[0] = pow(euler_orientation_standard_deviation_packet.standard_deviation[0], 2);
		imu_msg_.orientation_covariance[4] = pow(euler_orientation_standard_deviation_packet.standard_deviation[1], 2);
		imu_msg_.orientation_covariance[8] = pow(euler_orientation_standard_deviation_packet.standard_deviation[2], 2);
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
}

/**
 * @brief Function to decode the NED Velocity ANPP Packet (ANPP.35).
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::nedVelocityRosDecoder(an_packet_t* an_packet) {
	ned_velocity_packet_t ned_velocity_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);

	if (decode_ned_velocity_packet(&ned_velocity_packet, an_packet) == 0) {
		ned_velocity_msg_.header.stamp = this->get_clock()->now();
		ned_velocity_msg_.header.frame_id = frame_id_;
		ned_velocity_msg_.anpp_header.header_lrc = an_packet->header[0];
		ned_velocity_msg_.anpp_header.id = an_packet->id;
		ned_velocity_msg_.anpp_header.length = an_packet->length;
		ned_velocity_msg_.anpp_header.crc = an_packet->header[3] | (an_packet->header[4] << 8);

		// Native NED nav frame - intentionally not rotated here. The Option A bridge package
		// rotates this together with velocity_std_dev's covariance in a single operation, using
		// the current orientation, so the mean and covariance can never end up inconsistent.
		ned_velocity_msg_.velocity.north = ned_velocity_packet.velocity[0];
		ned_velocity_msg_.velocity.east = ned_velocity_packet.velocity[1];
		ned_velocity_msg_.velocity.down = ned_velocity_packet.velocity[2];
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
}

/**
 * @brief Function to decode the Body Acceleration ANPP Packet (ANPP.38).
 *
 * Also feeds imu_msg_.linear_acceleration with the FRD->FLU fix applied (confirmed bug, S0 spike):
 * static 180deg rotation about X, (x, -y, -z).
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::bodyAccelRosDecoder(an_packet_t* an_packet) {
	body_acceleration_packet_t body_acceleration_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);

	if (decode_body_acceleration_packet(&body_acceleration_packet, an_packet) == 0) {
		body_acceleration_msg_.header.stamp = this->get_clock()->now();
		body_acceleration_msg_.header.frame_id = frame_id_;
		body_acceleration_msg_.anpp_header.header_lrc = an_packet->header[0];
		body_acceleration_msg_.anpp_header.id = an_packet->id;
		body_acceleration_msg_.anpp_header.length = an_packet->length;
		body_acceleration_msg_.anpp_header.crc = an_packet->header[3] | (an_packet->header[4] << 8);

		// Native FRD frame, unflipped - raw/diagnostic topic.
		body_acceleration_msg_.body_acceleration.x = body_acceleration_packet.acceleration[0];
		body_acceleration_msg_.body_acceleration.y = body_acceleration_packet.acceleration[1];
		body_acceleration_msg_.body_acceleration.z = body_acceleration_packet.acceleration[2];
		body_acceleration_msg_.g_force = body_acceleration_packet.g_force;

		// imu_msg_: FRD->FLU fix applied.
		imu_msg_.linear_acceleration.x = body_acceleration_packet.acceleration[0];
		imu_msg_.linear_acceleration.y = -body_acceleration_packet.acceleration[1];
		imu_msg_.linear_acceleration.z = -body_acceleration_packet.acceleration[2];
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
}

/**
 * @brief Function to decode the Euler Orientation ANPP Packet (ANPP.39).
 *
 * Also computes the shared orientation_ quaternion (roll/pitch unchanged, yaw_enu = pi/2 - heading
 * per REP103, moved here from systemStateRosDecoder()) and feeds imu_msg_.orientation and
 * pose_msg_.orientation.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::eulerOrientationRosDecoder(an_packet_t* an_packet) {
	euler_orientation_packet_t euler_orientation_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);

	if (decode_euler_orientation_packet(&euler_orientation_packet, an_packet) == 0) {
		euler_orientation_msg_.header.stamp = this->get_clock()->now();
		euler_orientation_msg_.header.frame_id = frame_id_;
		euler_orientation_msg_.anpp_header.header_lrc = an_packet->header[0];
		euler_orientation_msg_.anpp_header.id = an_packet->id;
		euler_orientation_msg_.anpp_header.length = an_packet->length;
		euler_orientation_msg_.anpp_header.crc = an_packet->header[3] | (an_packet->header[4] << 8);

		// Native NED body->nav convention, unremapped - raw/diagnostic topic.
		euler_orientation_msg_.orientation.roll = euler_orientation_packet.orientation[0];
		euler_orientation_msg_.orientation.pitch = euler_orientation_packet.orientation[1];
		euler_orientation_msg_.orientation.heading = euler_orientation_packet.orientation[2];

		// Using the RPY orientation as done by cosama (moved from systemStateRosDecoder/Packet 20)
		orientation_.setRPY(
			euler_orientation_packet.orientation[0],
			euler_orientation_packet.orientation[1],
			M_PI/2.0f - euler_orientation_packet.orientation[2] // REP 103
		);
		imu_msg_.orientation.x = orientation_[0];
		imu_msg_.orientation.y = orientation_[1];
		imu_msg_.orientation.z = orientation_[2];
		imu_msg_.orientation.w = orientation_[3];

		pose_msg_.orientation.x = orientation_[0];
		pose_msg_.orientation.y = orientation_[1];
		pose_msg_.orientation.z = orientation_[2];
		pose_msg_.orientation.w = orientation_[3];
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
}

/**
 * @brief Function to decode the Quaternion Orientation ANPP Packet (ANPP.40).
 *
 * NOT frame-corrected - published for diagnostics/future validation only, see
 * QuaternionOrientation.msg. The device transmits QS/QX/QY/QZ scalar-first; mapped here to the
 * scalar-last geometry_msgs/Quaternion (QS->w, QX->x, QY->y, QZ->z).
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::quaternionOrientRosDecoder(an_packet_t* an_packet) {
	quaternion_orientation_packet_t quaternion_orientation_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);

	if (decode_quaternion_orientation_packet(&quaternion_orientation_packet, an_packet) == 0) {
		quaternion_orientation_msg_.header.stamp = this->get_clock()->now();
		quaternion_orientation_msg_.header.frame_id = frame_id_;
		quaternion_orientation_msg_.anpp_header.header_lrc = an_packet->header[0];
		quaternion_orientation_msg_.anpp_header.id = an_packet->id;
		quaternion_orientation_msg_.anpp_header.length = an_packet->length;
		quaternion_orientation_msg_.anpp_header.crc = an_packet->header[3] | (an_packet->header[4] << 8);

		quaternion_orientation_msg_.orientation.w = quaternion_orientation_packet.orientation[0];
		quaternion_orientation_msg_.orientation.x = quaternion_orientation_packet.orientation[1];
		quaternion_orientation_msg_.orientation.y = quaternion_orientation_packet.orientation[2];
		quaternion_orientation_msg_.orientation.z = quaternion_orientation_packet.orientation[3];
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
}

/**
 * @brief Function to decode the Angular Velocity ANPP Packet (ANPP.42).
 *
 * Also feeds imu_msg_.angular_velocity with the FRD->FLU fix applied: (x, -y, -z).
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::angularVelRosDecoder(an_packet_t* an_packet) {
	angular_velocity_packet_t angular_velocity_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);

	if (decode_angular_velocity_packet(&angular_velocity_packet, an_packet) == 0) {
		angular_velocity_msg_.header.stamp = this->get_clock()->now();
		angular_velocity_msg_.header.frame_id = frame_id_;
		angular_velocity_msg_.anpp_header.header_lrc = an_packet->header[0];
		angular_velocity_msg_.anpp_header.id = an_packet->id;
		angular_velocity_msg_.anpp_header.length = an_packet->length;
		angular_velocity_msg_.anpp_header.crc = an_packet->header[3] | (an_packet->header[4] << 8);

		// Native FRD frame, unflipped - raw/diagnostic topic.
		angular_velocity_msg_.angular_velocity.x = angular_velocity_packet.angular_velocity[0];
		angular_velocity_msg_.angular_velocity.y = angular_velocity_packet.angular_velocity[1];
		angular_velocity_msg_.angular_velocity.z = angular_velocity_packet.angular_velocity[2];

		// imu_msg_: FRD->FLU fix applied.
		imu_msg_.angular_velocity.x = angular_velocity_packet.angular_velocity[0];
		imu_msg_.angular_velocity.y = -angular_velocity_packet.angular_velocity[1];
		imu_msg_.angular_velocity.z = -angular_velocity_packet.angular_velocity[2];
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
}

/**
 * @brief Function to decode the Angular Acceleration ANPP Packet (ANPP.43).
 *
 * Published raw (native FRD, unflipped) for diagnostics - not consumed elsewhere in the locked-in
 * pipeline.
 *
 * @param an_packet a pointer to an an_packet_t object which will be decoded.
 */
void Driver::angularAccelRosDecoder(an_packet_t* an_packet) {
	angular_acceleration_packet_t angular_acceleration_packet;
	std::unique_lock<std::mutex> lock(messages_mutex_);

	if (decode_angular_acceleration_packet(&angular_acceleration_packet, an_packet) == 0) {
		angular_acceleration_msg_.header.stamp = this->get_clock()->now();
		angular_acceleration_msg_.header.frame_id = frame_id_;
		angular_acceleration_msg_.anpp_header.header_lrc = an_packet->header[0];
		angular_acceleration_msg_.anpp_header.id = an_packet->id;
		angular_acceleration_msg_.anpp_header.length = an_packet->length;
		angular_acceleration_msg_.anpp_header.crc = an_packet->header[3] | (an_packet->header[4] << 8);

		angular_acceleration_msg_.angular_acceleration.x = angular_acceleration_packet.angular_acceleration[0];
		angular_acceleration_msg_.angular_acceleration.y = angular_acceleration_packet.angular_acceleration[1];
		angular_acceleration_msg_.angular_acceleration.z = angular_acceleration_packet.angular_acceleration[2];
	}
	msg_write_done_ = true;
	msg_cv_.notify_one();
}

}// namespace adnav





