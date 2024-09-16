#include "bsw/buffer.hpp"

namespace bsw{
namespace interface{

using namespace dbc;
void Buffer::init(std::unordered_map<uint32_t, bsw::interface::can_msg_t>& buffer, const int& channel){
	//* AD-CAN
	if(channel == CAN_CH2){
		initBuffer(buffer, ad_can::DoorControl);
		initBuffer(buffer, ad_can::TurnSignalControl);
		initBuffer(buffer, ad_can::OperationControl);
		initBuffer(buffer, ad_can::BrainState);
		initBuffer(buffer, ad_can::AutonomousState);
		initBuffer(buffer, ad_can::LateralControl);
		initBuffer(buffer, ad_can::LongitudinalControl);
		initBuffer(buffer, ad_can::GearControl);
		initBuffer(buffer, ad_can::RemoteControlStatus);
		initBuffer(buffer, ad_can::DrivingLoggerInfo0);
		initBuffer(buffer, ad_can::DrivingLoggerInfo1);
	}
	if(channel == CAN_CH1){
		//* V-CAN
		initBuffer(buffer, v_can::TurnSignalControl);
		initBuffer(buffer, v_can::LongitudinalControl);
		initBuffer(buffer, v_can::GearControl);
		initBuffer(buffer, v_can::LateralControl);
		initBuffer(buffer, v_can::DoorControl);
		initBuffer(buffer, v_can::GearState);
		initBuffer(buffer, v_can::DoorState);
		initBuffer(buffer, v_can::TurnSignalState);
		initBuffer(buffer, v_can::LateralState);
		initBuffer(buffer, v_can::LongitudinalState);
		initBuffer(buffer, v_can::IgnitionState);
		initBuffer(buffer, v_can::IgnitionInfo);
		initBuffer(buffer, v_can::LongitudinalInfo);
		// 지원님의 알고리즘을 위한 추가 메시지
		initBuffer(buffer, v_can::WheelInfo);// wheel speed
		initBuffer(buffer, v_can::DynamicInfo); // acceleration
		initBuffer(buffer, v_can::SteeringInfo); // lateral info

	}
	if(channel == CAN_CH3){
		//* CAMERA-CAN
		initBuffer(buffer, camera::FrontCamObject01);
		initBuffer(buffer, camera::FrontCamObject02);
		initBuffer(buffer, camera::FrontCamObject03);
	}
};


}
}
