#include "rte.hpp"

using namespace rte;
using namespace dbc;

Rte::Rte(bsw::interface::CanTransfer* can_transfer,
		 bsw::interface::IoFunction* io_function)
		:can_transfer_(can_transfer),
		 io_function_(io_function),
		 init_flag_(false),
		 boot_counter_(0){
	crc_calc_.initCrc8(0x1d);
	crc_calc_.initCrc16(0x1021);
};

Rte::Rte(bsw::interface::CanTransfer* can_transfer,
		 bsw::interface::IoFunction* io_function,
		 bsw::interface::AdcReader* adc_reader,
		 bsw::interface::DacWriter* dac_writer)
		:can_transfer_(can_transfer),
		 io_function_(io_function),
		 adc_reader_(adc_reader),
		 dac_writer_(dac_writer),
		 init_flag_(false),
		 boot_counter_(0){
	crc_calc_.initCrc8(0x1d);
	crc_calc_.initCrc16(0x1021);
};

void Rte::initRte(void){
	can_transfer_->initMessageFilters();
 	io_function_->detachCanBus();
	can_transfer_->turnOnBus(CAN_CH1, true); // V CAN
 	can_transfer_->turnOnBus(CAN_CH2, true); // AD CAN
 	can_transfer_->turnOnBus(CAN_CH3, true); // CAMERA CAN


	can_transfer_->setBypassMode(true);
	init_flag_ = true;
}

void Rte::task20ms(void){
	// 20ms task

	//* get vehicle_state
	v_can::WheelInfo msg_speed_v = getBufferedMessage(CAN_CH1, v_can::WheelInfo);
	v_can::SteeringInfo msg_steer = getBufferedMessage(CAN_CH1, v_can::SteeringInfo);
	v_can::DynamicInfo msg_lon_v = getBufferedMessage(CAN_CH1, v_can::DynamicInfo);

	//* get longitudinal control frame from buffer from ADCAN
	ad_can::LongitudinalControl msg_lon_ctrl_ad = getBufferedMessage(CAN_CH2, ad_can::LongitudinalControl);

	//* get object information
	camera::FrontCamObject01 msg_cam_obj01 = getBufferedMessage(CAN_CH3, camera::FrontCamObject01);
	camera::FrontCamObject02 msg_cam_obj02 = getBufferedMessage(CAN_CH3, camera::FrontCamObject02);
	camera::FrontCamObject03 msg_cam_obj03 = getBufferedMessage(CAN_CH3, camera::FrontCamObject03);
//	camera::FrontCamObject04 msg_cam_obj04 = getBufferedMessage(CAN_CH3, camera::FrontCamObject04);
//	camera::FrontCamObject05 msg_cam_obj05 = getBufferedMessage(CAN_CH3, camera::FrontCamObject05);

	//* get acceleration value from control message
	auto target_accel = msg_lon_ctrl_ad.target_acceleration();

	auto vehicle_accel = msg_lon_v.long_acceleration();
	auto vehicle_speed = (msg_speed_v.wheel_speed_fl() + msg_speed_v.wheel_speed_fr())/2.;
	auto vehicle_steer = msg_steer.steering_angle();

	// 각 FrontCamObject에 대한 정보를 저장하는 배열
	std::vector<ObjectInfo> objectInfos;

	ObjectInfo info_01;
	info_01.classification = msg_cam_obj01.classification_object_01();
	info_01.relative_X_position = msg_cam_obj01.relative_X_position_01();
	info_01.relative_Y_position = msg_cam_obj01.relative_Y_position_01();
	info_01.relative_X_velocity = msg_cam_obj01.relative_X_velocity_01();
	info_01.relative_Y_velocity = msg_cam_obj01.relative_Y_velocity_01();

	ObjectInfo info_02;
	info_02.classification = msg_cam_obj01.classification_object_02();
	info_02.relative_X_position = msg_cam_obj01.relative_X_position_02();
	info_02.relative_Y_position = msg_cam_obj01.relative_Y_position_02();
	info_02.relative_X_velocity = msg_cam_obj01.relative_X_velocity_02();
	info_02.relative_Y_velocity = msg_cam_obj01.relative_Y_velocity_02();

	ObjectInfo info_03;
	info_03.classification = msg_cam_obj02.classification_object_03();
	info_03.relative_X_position = msg_cam_obj02.relative_X_position_03();
	info_03.relative_Y_position = msg_cam_obj02.relative_Y_position_03();
	info_03.relative_X_velocity = msg_cam_obj02.relative_X_velocity_03();
	info_03.relative_Y_velocity = msg_cam_obj02.relative_Y_velocity_03();

//	ObjectInfo info_04;
//	info_04.classification = msg_cam_obj02.classification_object_04();
//	info_04.relative_X_position = msg_cam_obj02.relative_X_position_04();
//	info_04.relative_Y_position = msg_cam_obj02.relative_Y_position_04();
//	info_04.relative_X_velocity = msg_cam_obj02.relative_X_velocity_04();
//	info_04.relative_Y_velocity = msg_cam_obj02.relative_Y_velocity_04();
//
//	ObjectInfo info_05;
//	info_05.classification = msg_cam_obj03.classification_object_05();
//	info_05.relative_X_position = msg_cam_obj03.relative_X_position_05();
//	info_05.relative_Y_position = msg_cam_obj03.relative_Y_position_05();
//	info_05.relative_X_velocity = msg_cam_obj03.relative_X_velocity_05();
//	info_05.relative_Y_velocity = msg_cam_obj03.relative_Y_velocity_05();
//
//	ObjectInfo info_06;
//	info_06.classification = msg_cam_obj03.classification_object_06();
//	info_06.relative_X_position = msg_cam_obj03.relative_X_position_06();
//	info_06.relative_Y_position = msg_cam_obj03.relative_Y_position_06();
//	info_06.relative_X_velocity = msg_cam_obj03.relative_X_velocity_06();
//	info_06.relative_Y_velocity = msg_cam_obj03.relative_Y_velocity_06();

	objectInfos.push_back(info_01);
	objectInfos.push_back(info_02);
	objectInfos.push_back(info_03);
//	objectInfos.push_back(info_04);
//	objectInfos.push_back(info_05);
//	objectInfos.push_back(info_06);


	//* set signal value for your application
	// yourApp.setInput(target_accel, ...);
	speed_filter::CollisionAvoidanceFilter speed_filter_(vehicle_speed,
														 vehicle_steer,
														 vehicle_accel,
														 target_accel,
														 objectInfos);

	//* run application
	speed_filter_.run();

	//* get output value from application
	auto output = speed_filter_.getOutput();

	//* init longitudinal control frame for VCAN
	v_can::LongitudinalControl msg_lon_ctrl_v;

	//* set target acceleration value from your application
	msg_lon_ctrl_v.target_acceleration(output);

	//* send frame to VCAN
	sendMessage(CAN_CH1, msg_lon_ctrl_v);
} //* end of task20ms

void Rte::task100ms(void){
	// 100ms task

} //* end of task100ms


void Rte::task200ms(void){
	// 200ms task
} //* end of task200ms

uint8_t Rte::checkValueInQueue(std::vector<uint8_t> queue){
	for(auto& item : queue){
		if(item != 0) return 1;
	}
	return 0;
}

void Rte::pushQueue(std::vector<uint8_t>& queue, uint8_t value){
	queue.push_back(value);
	queue.erase(queue.begin());
}
