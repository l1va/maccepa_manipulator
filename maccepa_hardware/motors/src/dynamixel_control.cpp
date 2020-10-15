#include <motors/dynamixel_control.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void DynamixelMACCEPA::chatterPositionCallback(const maccepa_msgs::MotorsPose& msg)
{
	dxl_goal_position[0] = msg.motor_1_pose / M_PI / 2 * VALUES_PER_REVOLUTE + dxl_init_position[0];
	dxl_goal_position[1] = msg.motor_2_pose / M_PI / 2 * VALUES_PER_REVOLUTE + dxl_init_position[1];
	dxl_goal_position[2] = msg.motor_3_pose / M_PI / 2 * VALUES_PER_REVOLUTE + dxl_init_position[2];
	dxl_goal_position[3] = msg.motor_4_pose / M_PI / 2 * VALUES_PER_REVOLUTE + dxl_init_position[3];

	setPosition(dxl_goal_position);
}

void DynamixelMACCEPA::chatterVelocityCallback(const maccepa_msgs::MotorsVel& msg)
{
	dxl_goal_velocity[0] = msg.motor_1_vel / RAD_PER_SEC;
	dxl_goal_velocity[1] = msg.motor_2_vel / RAD_PER_SEC;
	dxl_goal_velocity[2] = msg.motor_3_vel / RAD_PER_SEC;
	dxl_goal_velocity[3] = msg.motor_4_vel / RAD_PER_SEC;

	setVelocity(dxl_goal_velocity);
}

DynamixelMACCEPA::DynamixelMACCEPA(const ros::NodeHandle& node)
{
	node.getParam("dynamixel_control/port_adress",port);

	node.getParam("dynamixel_control/init_position_1",dxl_init_position[0]);
	node.getParam("dynamixel_control/init_position_2",dxl_init_position[1]);
	node.getParam("dynamixel_control/init_position_3",dxl_init_position[2]);
	node.getParam("dynamixel_control/init_position_4",dxl_init_position[3]);

	node.getParam("dynamixel_control/init_offset_1",dxl_offset[0]);
	node.getParam("dynamixel_control/init_offset_2",dxl_offset[1]);
	node.getParam("dynamixel_control/init_offset_3",dxl_offset[2]);
	node.getParam("dynamixel_control/init_offset_4",dxl_offset[3]);

	node.getParam("dynamixel_control/velocity_mode",velocity_mode);
	
	portHandler = dynamixel::PortHandler::getPortHandler(port.c_str());
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Initialize GroupSyncWrite instance
	groupSyncVelWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, GOAL_VELOCITY_ADRESS, 4);
	groupSyncPoseWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, GOAL_POSITION_ADRESS, 4);
	groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, PRESENT_POSITION_ADRESS, 4);

	// Open port
	if (portHandler->openPort()) {
		printf("Succeeded to open the port!\n");
	} else {
		throw std::runtime_error("Failed to open the port!\n");
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE)) {
		printf("Succeeded to change the baudrate!\n");
	} else {
		throw std::runtime_error("Failed to change the baudrate!\n");
	}  

	initSettings();

	for(int i = 0; i < DXLS.size(); i++)
	{
		// Add parameter storage for Dynamixel#1 present position value
		bool dxl_addparam_result = groupSyncRead->addParam(DXLS[i].id);
		if (dxl_addparam_result != true) 
		{
			fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXLS[i].id);
			throw std::system_error();
		}
	}
}

void DynamixelMACCEPA::checkCommAnswer(int dxl_comm_result, int dxl_error, int dxl_id){
	if (dxl_comm_result != COMM_SUCCESS) 
	{
		std::string tmp = packetHandler->getTxRxResult(dxl_comm_result) + '\n';
		throw std::runtime_error(tmp);
	} 
	else if (dxl_error != 0) 
	{
		std::string tmp = packetHandler->getRxPacketError(dxl_error) + '\n';
		throw std::runtime_error(tmp);
	}
	else
  	{
    	printf("Dynamixel#%d has been successfully connected \n", dxl_id);
  	}
}

void DynamixelMACCEPA::setVelocity(const std::array<int,4>& dxl_goal_velocity) 
{
	int dxl_comm_result = COMM_TX_FAIL;               // Communication result
	bool dxl_addparam_result = false;
	for(int i = 0; i < DXLS.size(); i++){
		int vel = dxl_goal_velocity[i];
		if (abs(dxl_goal_velocity[i]) > MAX_VELOCITY){
			vel = sgn(dxl_goal_velocity[i])*MAX_VELOCITY;
		}
		DXLS[i].goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(vel));
		DXLS[i].goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(vel));
		DXLS[i].goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(vel));
		DXLS[i].goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(vel));

		dxl_addparam_result = groupSyncVelWrite->addParam(DXLS[i].id, DXLS[i].goal_velocity);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXLS[i].id);
			throw std::system_error();
		}
	}
	// Syncwrite goal position
	dxl_comm_result = groupSyncVelWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear syncwrite parameter storage
	groupSyncVelWrite->clearParam();
}

void DynamixelMACCEPA::setPosition(const std::array<int,4>& dxl_goal_position) 
{
	int dxl_comm_result = COMM_TX_FAIL;               // Communication result
	bool dxl_addparam_result = false;                 // addParam result

	for(int i = 0; i < DXLS.size(); i++){
		DXLS[i].goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
		DXLS[i].goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
		DXLS[i].goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
		DXLS[i].goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

		dxl_addparam_result = groupSyncPoseWrite->addParam(DXLS[i].id, DXLS[i].goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXLS[i].id);
			throw std::system_error();
		}
	}
	// Syncwrite goal position
	dxl_comm_result = groupSyncPoseWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear syncwrite parameter storage
	groupSyncPoseWrite->clearParam();
}

void DynamixelMACCEPA::updatePresentPosition(maccepa_msgs::MotorsPose& pose){
	uint8_t dxl_error = 0;
	int dxl_comm_result = groupSyncRead->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
    else 
    {
		for(int i = 0; i < DXLS.size(); i++)
		{
			if (groupSyncRead->getError(DXLS[i].id, &dxl_error))
			{
				printf("[ID:%03d] %s\n", DXLS[i].id, packetHandler->getRxPacketError(dxl_error));
			}
			bool dxl_getdata_result = groupSyncRead->isAvailable(DXLS[i].id, PRESENT_POSITION_ADRESS, 4);
    		if (dxl_getdata_result != true)
    		{
    			fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXLS[i].id);
				throw std::system_error();
    		}
			dxl_present_position[i] = groupSyncRead->getData(DXLS[i].id, PRESENT_POSITION_ADRESS, 4);
		}
    }

	pose.motor_1_pose = static_cast<float>(dxl_present_position[0] - dxl_init_position[0]) / VALUES_PER_REVOLUTE * 2 * M_PI;
	pose.motor_2_pose = static_cast<float>(dxl_present_position[1] - dxl_init_position[1]) / VALUES_PER_REVOLUTE * 2 * M_PI;
	pose.motor_3_pose = static_cast<float>(dxl_present_position[2]- dxl_init_position[2]) / VALUES_PER_REVOLUTE * 2 * M_PI;
	pose.motor_4_pose = static_cast<float>(dxl_present_position[3] - dxl_init_position[3]) / VALUES_PER_REVOLUTE * 2 * M_PI;
}


void DynamixelMACCEPA::initSettings()
{
	uint8_t dxl_error = 0;
	int dxl_comm_result = 0;
	for(int i = 0; i < DXLS.size(); i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXLS[i].id, SET_MODE_ADRESS, 3, &dxl_error);
		checkCommAnswer(dxl_comm_result, dxl_error, DXLS[i].id);
		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXLS[i].id, OFFSET_ADRESS, dxl_offset[i], &dxl_error);
		packetHandler->write1ByteTxRx(portHandler, DXLS[i].id, TORQUE_ENABLE_ADRESS, 1, &dxl_error);
		packetHandler->write4ByteTxRx(portHandler, DXLS[i].id, GOAL_POSITION_ADRESS, dxl_init_position[i], &dxl_error);
	}
	
	//delay for changing position
	sleep(2);
	if(velocity_mode){
		for(int i = 0; i < DXLS.size(); i++)
		{	
			packetHandler->write1ByteTxRx(portHandler, DXLS[i].id, TORQUE_ENABLE_ADRESS, 0, &dxl_error);
			packetHandler->write1ByteTxRx(portHandler, DXLS[i].id, SET_MODE_ADRESS, 1, &dxl_error);
			packetHandler->write1ByteTxRx(portHandler, DXLS[i].id, TORQUE_ENABLE_ADRESS, 1, &dxl_error);
		}
		printf("Velocity mode on\n");
	} else
	{
		printf("Velocity mode off\n");
	}

}

DynamixelMACCEPA::~DynamixelMACCEPA() 
{
	delete groupSyncRead;
	delete groupSyncVelWrite;
	delete groupSyncPoseWrite;

	uint8_t dxl_error = 0;
	int dxl_comm_result = COMM_TX_FAIL;

	for(const auto &dxl: DXLS ) {
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl.id, TORQUE_ENABLE_ADRESS, 0, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) {
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		} else if (dxl_error != 0) {
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}
	}

	// Close port
	portHandler->closePort();
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "dynamixel_control");
  	ros::NodeHandle n;

	DynamixelMACCEPA driver(n);
	ros::Subscriber sub;
	if(driver.velocityMode())
	{
		sub = n.subscribe("set_velocity", 100, &DynamixelMACCEPA::chatterVelocityCallback, &driver);
	}
  	else
	{
		sub = n.subscribe("set_position", 100, &DynamixelMACCEPA::chatterPositionCallback, &driver);
	}
	
	ros::Publisher pose_pub = n.advertise<maccepa_msgs::MotorsPose>("states", 100);
  	ros::Rate loop_rate(20);

	maccepa_msgs::MotorsPose present_pose;

  	while (ros::ok())
  	{
		driver.updatePresentPosition(present_pose);
  	  	pose_pub.publish(present_pose);

  	  	ros::spinOnce();
  	  	loop_rate.sleep();
  	}

  return 0;
}