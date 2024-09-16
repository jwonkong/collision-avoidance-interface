#include "bsw/interface/can_transfer.hpp"
#include "app/validation.h"
#include <assert.h>
#include <string.h>

#include "main.h"

extern TIM_HandleTypeDef htim3;

static bool can_sending_[bsw::interface::CAN_CH_LEN];
static uint8_t tx_event_buf_idx[bsw::interface::CAN_CH_LEN] = {false, false, false};

static uint8_t prev_error_code[bsw::interface::CAN_CH_LEN];
static uint8_t prev_data_error_code[bsw::interface::CAN_CH_LEN];

static osMessageQId queue_handle[bsw::interface::CAN_CH_LEN];
static uint8_t queue_buffer[bsw::interface::CAN_CH_LEN][bsw::interface::LOG_MSG_LEN * sizeof(bsw::interface::can_msg_t)];
static osStaticMessageQDef_t queue_control_block[bsw::interface::CAN_CH_LEN];

static uint32_t queue_count_put[bsw::interface::CAN_CH_LEN];
static uint32_t queue_count_get[bsw::interface::CAN_CH_LEN];
static uint32_t queue_count_get_prev[bsw::interface::CAN_CH_LEN];
static bool queue_count_get_busy[bsw::interface::CAN_CH_LEN];

static osThreadId half_signal_thread[bsw::interface::CAN_CH_LEN];
static int32_t half_signal_value[bsw::interface::CAN_CH_LEN];

static FDCAN_HandleTypeDef *can_dev[bsw::interface::CAN_CH_LEN];

static uint32_t local_timestamp;

static osMessageQId queue_error_handle[bsw::interface::CAN_CH_LEN];
static uint8_t queue_error_buffer[bsw::interface::CAN_CH_LEN][bsw::interface::LOG_MSG_LEN * sizeof(bsw::interface::can_msg_t)];
static osStaticMessageQDef_t queue_error_control_block[bsw::interface::CAN_CH_LEN];

static bool bypass_ = false;
static bool bus_on[bsw::interface::CAN_CH_LEN];
bsw::interface::MsgBuffer* buffer_;

std::unordered_map<std::string, std::vector<uint32_t>> case_filters_;
std::unordered_map<std::string, bool> is_case_;

/**
 * Queue definition for CAN ISR-TASK communication.
 * It is based on a shared queue of size 100.
 */
#define FDCAN_QUEUE_SIZE bsw::interface::LOG_MSG_LEN
bsw::interface::rx_queue_msg_t rx_queued_msg_[FDCAN_QUEUE_SIZE];
uint8_t rx_queue_pointer_ = 0;
QueueHandle_t can_rx_queue;

bsw::interface::tx_queue_msg_t tx_queued_msg_[FDCAN_QUEUE_SIZE];
uint8_t tx_queue_pointer_ = 0;
QueueHandle_t can_tx_queue;

/**
 * CAN RX Task variables
 */
static TaskHandle_t canRxTaskHandle;
static TaskHandle_t canTxTaskHandle;
#define CAN_RX_TASK_PRIORITY 11 // This equals 3 in osPriorityRealtime for FreeRTOS.
#define CAN_TX_TASK_PRIORITY 12 // This equals 3 in osPriorityRealtime for FreeRTOS.

/**
 * configurations
 */
bool bus2device_bypass_ = false;
bool filtering_message_ = false;

bool isFiltered(const std::string& c, const uint32_t& id){
	if(!is_case_[c]) return false;

	auto list = case_filters_[c];
	return (std::find(list.begin(), list.end(), id) != list.end());
}

static size_t size_from_dlc_code(uint32_t dlc_code) {
	switch (dlc_code) {
	case FDCAN_DLC_BYTES_0:
		return 0;
	case FDCAN_DLC_BYTES_1:
		return 1;
	case FDCAN_DLC_BYTES_2:
		return 2;
	case FDCAN_DLC_BYTES_3:
		return 3;
	case FDCAN_DLC_BYTES_4:
		return 4;
	case FDCAN_DLC_BYTES_5:
		return 5;
	case FDCAN_DLC_BYTES_6:
		return 6;
	case FDCAN_DLC_BYTES_7:
		return 7;
	case FDCAN_DLC_BYTES_8:
		return 8;
	case FDCAN_DLC_BYTES_12:
		return 12;
	case FDCAN_DLC_BYTES_16:
		return 16;
	case FDCAN_DLC_BYTES_20:
		return 20;
	case FDCAN_DLC_BYTES_24:
		return 24;
	case FDCAN_DLC_BYTES_32:
		return 32;
	case FDCAN_DLC_BYTES_48:
		return 48;
	case FDCAN_DLC_BYTES_64:
		return 64;
	default:
		return 0;
	}
}

static uint32_t timestamp_expend(uint32_t event_timestamp) {
	uint32_t current_time;
	uint32_t high_current_time1;
	uint32_t high_current_time2;
	uint32_t high_current_time;
	uint32_t time_diff;

	high_current_time1 = local_timestamp << 16;
	current_time = htim3.Instance->CNT;
	high_current_time2 = local_timestamp << 16;
	if (current_time < 0x8000)
		high_current_time = high_current_time2;
	else
		high_current_time = high_current_time1;
	if (current_time > event_timestamp)
		time_diff = current_time - event_timestamp;
	else
		time_diff = current_time - event_timestamp + 0x10000;

	return (high_current_time | current_time) - time_diff;
}

void half_check(uint8_t can_ch) {
	if (half_signal_thread[can_ch] == NULL) return;

	uint32_t get_count;
	uint32_t put_count = queue_count_put[can_ch];
	uint32_t len;

	if (queue_count_get_busy[can_ch]) {
		get_count = queue_count_get_prev[can_ch];
	} else {
		get_count = queue_count_get[can_ch];
	}

	len = put_count - get_count;

	if (len > bsw::interface::LOG_MSG_LEN) return;

	if (len < (bsw::interface::LOG_MSG_LEN / 2)) return;
	if (osSignalSet(half_signal_thread[can_ch], half_signal_value[can_ch]) ==
			static_cast<int32_t>(0x80000000)) {
		half_signal_thread[can_ch] = NULL;
		return;
	}
}

void canRxTask(void const * argument) {
	for(;;) {
		vTaskSuspend ( NULL );

		bsw::interface::rx_queue_msg_t* queued_msg;
		BaseType_t xStatus;
		BaseType_t xTaskWokenByReceive = pdFALSE;

		while(uxQueueMessagesWaiting(can_rx_queue) > 0) {
			xStatus = xQueueReceiveFromISR(can_rx_queue, &queued_msg, &xTaskWokenByReceive);

			if(xStatus == pdPASS) {
				uint8_t can_ch = queued_msg->can_ch;
				FDCAN_RxHeaderTypeDef rx_header = queued_msg->rx_header;

				bsw::interface::can_msg_t msg;
				msg.can_id = rx_header.Identifier;
				if(rx_header.IdType == FDCAN_EXTENDED_ID) msg.can_id |= EFF_FLAG;
				msg.data_size = size_from_dlc_code(rx_header.DataLength);
				memcpy(msg.data, &queued_msg->rx_data[0], msg.data_size);
				msg.fd = rx_header.FDFormat == FDCAN_FD_CAN;
				msg.brs = rx_header.BitRateSwitch == FDCAN_BRS_ON;
				msg.timestamp = timestamp_expend(rx_header.RxTimestamp);
				msg.dir = DIR_RX;
				msg.is_completed = false;

				//* TODO : Check frame length for validation
				if(!bus2device_bypass_){
					if(msg.data_size != buffer_->getBuffer(can_ch, msg.can_id).data_size) {
						return;
					}
				}
				if(bus2device_bypass_){
					if(can_ch == CAN_CH2){
						//* bypass to CH2
						FDCAN_TxHeaderTypeDef tx_header;
						tx_header.Identifier = rx_header.Identifier;
						tx_header.IdType = rx_header.IdType;
						tx_header.TxFrameType = rx_header.RxFrameType;
						tx_header.ErrorStateIndicator = rx_header.ErrorStateIndicator;
						tx_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
						tx_header.BitRateSwitch = rx_header.BitRateSwitch;
						tx_header.FDFormat = rx_header.FDFormat;
						tx_header.DataLength = rx_header.DataLength;

						bool filtering_flag = filtering_message_? !(isFiltered("no-bypass", msg.can_id)) && !(isFiltered("ad", msg.can_id)) : true;

						if(filtering_flag){
							if(bypass_){
								tx_header.MessageMarker = tx_event_buf_idx[CAN_CH3];
								bsw::interface::tx_queue_msg_t* queued_msg = &tx_queued_msg_[tx_queue_pointer_];
								portENTER_CRITICAL();
								tx_queue_pointer_++;
								if(tx_queue_pointer_ >= FDCAN_QUEUE_SIZE) tx_queue_pointer_ = 0;
								portEXIT_CRITICAL();
								queued_msg->tx_header = tx_header;
								queued_msg->can_ch = CAN_CH3;
								memcpy(queued_msg->tx_data, msg.data, msg.data_size);

								TickType_t xTickToWait = 0;
								xQueueSend(can_tx_queue, &queued_msg, xTickToWait);
								if (eTaskGetState(canTxTaskHandle) == eSuspended) {
									vTaskResume(canTxTaskHandle);
								}
							}
							if(buffer_->isListed(CAN_CH3, msg.can_id)) buffer_->setBuffer(CAN_CH3, msg);
						}
						//* set buffer for the algorithm
						if(buffer_->isListed(can_ch, msg.can_id)) buffer_->setBuffer(can_ch, msg);
					}
					else if(can_ch == CAN_CH3){
						if(buffer_->isListed(can_ch, msg.can_id)) buffer_->setBuffer(can_ch, msg);
					}
					else{
						if(buffer_->isListed(can_ch, msg.can_id)) buffer_->setBuffer(can_ch, msg);
					}
				}
				else{
					if(buffer_->isListed(can_ch, msg.can_id)) buffer_->setBuffer(can_ch, msg);
				}
			}
		}
	}
}


//* replace HAL_FDCAN_TxEventFifoCallback
/**
 *  @brief 	replace HAL_FDCAN_TxEventFifoCallback
 *  @detail	Add FDCAN message to Tx Queue
 *  				handling every CAN message transfer from rte timer task and canRxTask
 *  @retval None
 */
void canTxTask(void const * argument) {
	int i = 0;
	for(;;) {
		vTaskSuspend ( NULL ); //* resume by canRxTask can cantransfer::send function
		bsw::interface::tx_queue_msg_t* queued_msg;
		BaseType_t xStatus;

		while(uxQueueMessagesWaiting(can_tx_queue) > 0) {
			xStatus = xQueueReceive(can_tx_queue, &queued_msg, 0);

			if(xStatus == pdPASS) {
				FDCAN_HandleTypeDef *hfdcan = can_dev[queued_msg->can_ch];
				FDCAN_TxHeaderTypeDef tx_header = queued_msg->tx_header;
				for(i = 0; i < (int)bsw::interface::RESEND_TRIAL; i++){
					auto ret = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, queued_msg->tx_data);
					if (ret == HAL_OK) break;
					else {
						if(bus_on[queued_msg->can_ch]) vTaskSuspend ( NULL );
						else break;
					}
				}
			}
		}
	}
}

/**
 *  @brief 	Rx FIFO 0 callback.
 *  @detail	Implementation of the weak HAL_FDCAN_RxFifo0Callback stm32g4xx_hal_fdcan.c
 *  		handling every CAN message from CAN channel message.
 *  @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         	the configuration information for the specified FDCAN.
 *  @param	RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
 *  @retval None
 */
using namespace dbc;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
		uint32_t RxFifo0ITs) {

		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
			bsw::interface::rx_queue_msg_t* queued_msg = &rx_queued_msg_[rx_queue_pointer_];
			rx_queue_pointer_++;
			if(rx_queue_pointer_ >= FDCAN_QUEUE_SIZE) rx_queue_pointer_=0;

			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &queued_msg->rx_header, &queued_msg->rx_data[0]);

			uint8_t can_ch;
			if (hfdcan->Instance == FDCAN1) {
				can_ch = CAN_CH1;
			} else if (hfdcan->Instance == FDCAN2) {
				can_ch = CAN_CH2;
			} else if (hfdcan->Instance == FDCAN3) {
				can_ch = CAN_CH3;
			}
			else {
				return;
			}
			//* live update handling
			if(can_ch == CAN_CH1 && queued_msg->rx_header.Identifier == FUC_LIVE_MSG_ID){
				dbc::boot::UpdateRequest msg_boot(&queued_msg->rx_data[0]);
				auto msg_bootid = msg_boot.boot_id();
				auto xcp_req = msg_boot.xcp_req();
				auto signature = msg_boot.signature();

				if(xcp_req == 0xFF && msg_bootid == bootid ){
					if(Comparehash(signature)==true)
					{
						//* reboot
						HAL_NVIC_SystemReset();
					} else {
						return;
					}
				} else {
					return;
				}
			}
			//* version request
			if (can_ch == CAN_CH1 && queued_msg->rx_header.Identifier == (uint32_t)dbc::boot::Id::VersionRequest){
				dbc::boot::VersionRequest msg_v_req(&queued_msg->rx_data[0]);
				auto req_boot_id = msg_v_req.target_boot_id();

				if (req_boot_id == bootid) {
					dbc::boot::VersionResponse res_data;
					res_data.response_boot_id((uint8_t)bootid);
					res_data.version_patch(version_P);
					res_data.version_minor(version_m);
					res_data.version_major(version_M);

					FDCAN_TxHeaderTypeDef tx_header;
					tx_header.Identifier = res_data.id;
					tx_header.IdType = FDCAN_STANDARD_ID;
					tx_header.TxFrameType = FDCAN_DATA_FRAME;
					tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
					tx_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
					tx_header.BitRateSwitch = FDCAN_BRS_OFF;
					tx_header.FDFormat = FDCAN_CLASSIC_CAN;
					tx_header.DataLength = bsw::interface::can_data_length[res_data.length];
					tx_header.MessageMarker = tx_event_buf_idx[CAN_CH1];

					auto ret = HAL_FDCAN_AddMessageToTxFifoQ(can_dev[CAN_CH1], &tx_header, &res_data.data[0]);

					if (ret == HAL_OK) {
						tx_event_buf_idx[CAN_CH1]++;
					} else {
						bsw::interface::can_msg_t msg_res;
						msg_res.can_id = res_data.id;
						msg_res.brs = res_data.flags.brs;
						msg_res.fd = res_data.flags.fd;
						msg_res.dir = DIR_TX;
						msg_res.data_size = res_data.length;
						memcpy(msg_res.data, &res_data.data[0], res_data.length);
						xQueueSendFromISR(queue_error_handle[CAN_CH1], &msg_res, 0);
					}
					return;
				} else {
					return;
				}
			} //* end of version request handling
			queued_msg->can_ch = can_ch;
			long xHigherPriorityTaskWoken = pdFALSE;
			xQueueSendFromISR(can_rx_queue, &queued_msg, &xHigherPriorityTaskWoken);
			BaseType_t xYieldRequired = xTaskResumeFromISR(canRxTaskHandle);
			portYIELD_FROM_ISR(xYieldRequired);
		}
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
	BaseType_t xYieldRequired = xTaskResumeFromISR(canTxTaskHandle);
	portYIELD_FROM_ISR(xYieldRequired);
}

using namespace bsw;
using namespace interface;

void CanTransfer::initUserBuffer(UserBuffer* user_buffer){
	buffer_ = new bsw::interface::MsgBuffer(user_buffer);
}

void CanTransfer::init(FDCAN_HandleTypeDef* hfdcan1, FDCAN_HandleTypeDef* hfdcan2, FDCAN_HandleTypeDef* hfdcan3) {
	size_t i;

	hfdcan1_= hfdcan1;
	hfdcan2_= hfdcan2;
	hfdcan3_= hfdcan3;

	for (i = 0; i < CAN_CH_LEN; i++) {
		tx_event_buf_idx[i] = 0;
		prev_error_code[i] = 0;
		prev_data_error_code[i] = 0;

		half_signal_thread[i] = NULL;
		half_signal_value[i] = 0;
		osMessageQStaticDef(queue, LOG_MSG_LEN, can_msg_t, queue_buffer[i],
				&queue_control_block[i]);
		queue_handle[i] = osMessageCreate(osMessageQ(queue), NULL);

		osMessageQStaticDef(queue_error, LOG_MSG_LEN, can_msg_t, queue_error_buffer[i],
				&queue_error_control_block[i]);
		queue_error_handle[i] = osMessageCreate(osMessageQ(queue_error), NULL);

		queue_count_put[i] = 0;
		queue_count_get[i] = 0;
		queue_count_get_prev[i] = 0;
		queue_count_get_busy[i] = false;
	}

	//* Init CAN message queue
	do {
		can_rx_queue = xQueueCreate(bsw::interface::LOG_MSG_LEN, sizeof(bsw::interface::rx_queue_msg_t *));
	} while(can_rx_queue == NULL);
	do {
		can_tx_queue = xQueueCreate(bsw::interface::LOG_MSG_LEN, sizeof(bsw::interface::tx_queue_msg_t *));
	} while(can_tx_queue == NULL);

	//* Create Task
	std::string can_rx_task_name = "canRxTask";
	xTaskCreate((TaskFunction_t) canRxTask, (char*)can_rx_task_name.c_str(), 1024, NULL, CAN_RX_TASK_PRIORITY, &canRxTaskHandle);

	std::string can_tx_task_name = "canTxTask";
	xTaskCreate((TaskFunction_t) canTxTask, (char*)can_tx_task_name.c_str(), 1024, NULL, CAN_TX_TASK_PRIORITY, &canTxTaskHandle);


	can_dev[CAN_CH1] = hfdcan1_;
	can_dev[CAN_CH2] = hfdcan2_;
	can_dev[CAN_CH3] = hfdcan3_;

	if (HAL_FDCAN_Init(hfdcan1_) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_Init(hfdcan2_) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_Init(hfdcan3_) != HAL_OK)
	{
		Error_Handler();
	}
	local_timestamp = 0;
}

/** @brief	Initialize each CAN channel message filter.
 *	@retval	None
 */
void CanTransfer::initMessageFilters(void){
	//* id filter
	setIdFilter(CAN_CH1, buffer_->getBufferIds(CAN_CH1));
	if(!bus2device_bypass_){
		setIdFilter(CAN_CH2, buffer_->getBufferIds(CAN_CH2));
	}
	setIdFilter(CAN_CH3, buffer_->getBufferIds(CAN_CH3));
}

/** @details	Set each CAN channel message filter.
 *  @ref : https://www.st.com/resource/en/application_note/an5348-introduction-to-fdcan-peripherals-for-stm32-product-classes-stmicroelectronics.pdf
 * 	@param	Target CAN channel
 * 	@param	List of filter ID from frame_buffer.cpp initBuffer frames
 *	@retval
 */
bool CanTransfer::setIdFilter(uint8_t can_ch, std::vector<uint32_t> id_list){
	int counter = 0;
	if((static_cast<int>(id_list.size()) % 2) != 0) id_list.push_back(0);

	for(int i = 1; i < static_cast<int>(id_list.size()); i+=2){
		FDCAN_FilterTypeDef sFilterConfig;
		//* Configure Rx filter
		sFilterConfig.IdType = FDCAN_STANDARD_ID;
		sFilterConfig.FilterIndex = counter++;
		//* Dual ID filter : standard filter element configuration
		sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
		//* Store Rx frame in FIFO 0 : standard filter type
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
		//* Store the messages with ID equal to id_list(i-1) and id_list(i)
		sFilterConfig.FilterID1 = id_list.at(i-1); // standard filter ID 1
		sFilterConfig.FilterID2 = id_list.at(i); // standard filter ID 2

		if(can_ch == CAN_CH1){
			if (HAL_FDCAN_ConfigFilter(hfdcan1_, &sFilterConfig) != HAL_OK) return false;
			if (HAL_FDCAN_ConfigGlobalFilter(hfdcan1_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) return false;
		}else if(can_ch == CAN_CH2){
			if (HAL_FDCAN_ConfigFilter(hfdcan2_, &sFilterConfig) != HAL_OK) return false;
			if (HAL_FDCAN_ConfigGlobalFilter(hfdcan2_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) return false;
		}else if(can_ch == CAN_CH3){
			if (HAL_FDCAN_ConfigFilter(hfdcan3_, &sFilterConfig) != HAL_OK) return false;
			if (HAL_FDCAN_ConfigGlobalFilter(hfdcan3_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) return false;
		}
	}
	return true;
}


int CanTransfer::turnOnBus(uint8_t can_ch, bool on) {
	if (on) {
		if (HAL_FDCAN_ActivateNotification(
				can_dev[can_ch], FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
			return -1;
		if (HAL_FDCAN_ActivateNotification(
				can_dev[can_ch], FDCAN_IT_TX_EVT_FIFO_NEW_DATA, 0) != HAL_OK)
			return -1;
		if (HAL_FDCAN_ActivateNotification(
				can_dev[can_ch], FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
			return -1;
		if (HAL_FDCAN_ActivateNotification(
						can_dev[can_ch], FDCAN_IT_BUS_OFF, 0) != HAL_OK)
					return -1;
		if (HAL_FDCAN_EnableTimestampCounter(
				can_dev[can_ch], FDCAN_TIMESTAMP_EXTERNAL) != HAL_OK)
			return -1;
		if (HAL_FDCAN_Start(can_dev[can_ch]) != HAL_OK) return -1;
		bus_on[can_ch] = true;
	} else {
		if (HAL_FDCAN_Stop(can_dev[can_ch]) != HAL_OK) return -1;
		bus_on[can_ch] = false;
	}

	return 0;
}

/** @brief	Set software bypass mode (CAN CH2 <-> CAN CH3)
 *  @param	bypass mode
 *
 */
void CanTransfer::setBypassMode(bool on){
	bypass_ = on;
}

void CanTransfer::setCaseFilter(const std::string& c, const std::vector<uint32_t> arr){
	case_filters_[c] = arr;
	setCase(c, false);
}

void CanTransfer::setCase(const std::string& c, const bool& flag){
	is_case_[c] = flag;
}

/**	@brief	Send CAN message to target channel
 * 	@param	Target CAN channel
 * 	@param	Target CAN message
 * 	@retval	-1|0
 * 	Fail to send message or empty message return -1, successful send return 0
 */
int CanTransfer::send(uint8_t can_ch, const can_msg_t& msg) {
	bsw::interface::tx_queue_msg_t* queued_msg = &tx_queued_msg_[tx_queue_pointer_];
	portENTER_CRITICAL();
	tx_queue_pointer_++;
	if(tx_queue_pointer_ >= FDCAN_QUEUE_SIZE) tx_queue_pointer_ = 0;
	portEXIT_CRITICAL();

	if (msg.data == NULL) return -1;

	queued_msg->tx_header.Identifier = msg.can_id & ~EFF_FLAG;

	if (msg.can_id & EFF_FLAG) {
		queued_msg->tx_header.IdType = FDCAN_EXTENDED_ID;
	} else {
		queued_msg->tx_header.IdType = FDCAN_STANDARD_ID;
	}
	queued_msg->tx_header.TxFrameType = FDCAN_DATA_FRAME;
	queued_msg->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	queued_msg->tx_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;

	if (msg.brs) {
		queued_msg->tx_header.BitRateSwitch = FDCAN_BRS_ON;
	} else {
		queued_msg->tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	}

	if (msg.fd) {
		queued_msg->tx_header.FDFormat = FDCAN_FD_CAN;
	} else {
		queued_msg->tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	}

	memset(queued_msg->tx_data, 0, sizeof(queued_msg->tx_data));
	memcpy(queued_msg->tx_data, msg.data, msg.data_size);
	queued_msg->tx_header.DataLength = can_data_length[msg.data_size];
	queued_msg->tx_header.MessageMarker = tx_event_buf_idx[can_ch];
	queued_msg->can_ch = can_ch;

	bool bus_off = false;
	bool error_passive = false;
	uint8_t error_code = 0;
	uint8_t data_error_code = 0;

	//* Check buss state
	busState(can_ch, &bus_off, &error_passive, &error_code, &data_error_code);
	if (bus_off){ //* if Bus Off, reset FDCAN handle
		HAL_FDCAN_DeInit(can_dev[can_ch]);
		HAL_FDCAN_Init(can_dev[can_ch]);

		if(bus2device_bypass_){
			if(can_ch != CAN_CH2){
				setIdFilter(can_ch, buffer_->getBufferIds(can_ch));
			}
		}
		else{
			setIdFilter(can_ch, buffer_->getBufferIds(can_ch));
		}
		turnOnBus(can_ch, true);
	}
	xQueueSend(can_tx_queue, &queued_msg, (TickType_t)0);
	if (eTaskGetState(canTxTaskHandle) == eSuspended) {
		vTaskResume(canTxTaskHandle);
	}
}

int CanTransfer::sendRegisterStatus(uint8_t can_ch) {
	can_msg_t msg;
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[64];
	HAL_StatusTypeDef ret;

	msg.brs = 0;
	msg.can_id = 0x100;
	msg.data_size = 8;
	msg.data[0] = hfdcan1_->ErrorCode;
	msg.data[1] = hfdcan2_->ErrorCode;
	msg.data[2] =  hfdcan3_->ErrorCode;

	msg.data[4] =  hfdcan1_->State;
	msg.data[5] =  hfdcan2_->State;
	msg.data[6] =  hfdcan3_->State;


	tx_header.Identifier = 0x100;
	tx_header.IdType = FDCAN_STANDARD_ID;

	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;

	tx_header.BitRateSwitch = FDCAN_BRS_OFF;

	tx_header.FDFormat = FDCAN_FD_CAN;

	memset(tx_data, 0, sizeof(tx_data));
	memcpy(tx_data, msg.data, msg.data_size);
	tx_header.DataLength = can_data_length[8];
	tx_header.MessageMarker = tx_event_buf_idx[can_ch];


	ret = HAL_FDCAN_AddMessageToTxFifoQ(can_dev[can_ch], &tx_header, tx_data);
	if (ret == HAL_OK) {
		tx_event_buf_idx[can_ch]++;
	} else {
		xQueueSendFromISR(queue_error_handle[can_ch], &msg, 0);
		return -1;
	}

	return 0;
}

int CanTransfer::sendWoBuffering(uint8_t can_ch, const can_msg_t& msg) {
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[64];
	HAL_StatusTypeDef ret;
	uint32_t i = 0;

	if (msg.data == NULL) return -1;

	tx_header.Identifier = msg.can_id & ~EFF_FLAG;
	if (msg.can_id & EFF_FLAG) {
		tx_header.IdType = FDCAN_EXTENDED_ID;
	} else {
		tx_header.IdType = FDCAN_STANDARD_ID;
	}
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;

	if (msg.brs) {
		tx_header.BitRateSwitch = FDCAN_BRS_ON;
	} else {
		tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	}

	if (msg.fd) {
		tx_header.FDFormat = FDCAN_FD_CAN;
	} else {
		tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	}

	memset(tx_data, 0, sizeof(tx_data));
	memcpy(tx_data, msg.data, msg.data_size);
	tx_header.DataLength = can_data_length[msg.data_size];
	tx_header.MessageMarker = tx_event_buf_idx[can_ch];

	for(i = 0; i < RESEND_TRIAL; i++){
		ret = HAL_FDCAN_AddMessageToTxFifoQ(can_dev[can_ch], &tx_header, tx_data);
		if (ret == HAL_OK) {
			tx_event_buf_idx[can_ch]++;
			break;
		}
	}

	return 0;
}

bool CanTransfer::recv(uint8_t can_ch, can_msg_t& msg) {
	if (xQueueReceive(queue_handle[can_ch], &msg, 0) == pdFALSE) return false;

	queue_count_get_busy[can_ch] = true;
	queue_count_get[can_ch]++;
	queue_count_get_busy[can_ch] = false;
	queue_count_get_prev[can_ch] = queue_count_get[can_ch];

	return true;
}

can_msg_t CanTransfer::getBuffer(uint8_t can_ch, uint32_t can_id){
	return buffer_->getBuffer(can_ch, can_id);
}

void CanTransfer::setBuffer(uint8_t can_ch, can_msg_t msg){
	buffer_->setBuffer(can_ch, msg);
}

uint8_t CanTransfer::getBufferSize(uint8_t can_ch){
	return static_cast<uint8_t>(buffer_->getBufferIds(can_ch).size());
}

uint8_t CanTransfer::getFilterSize(uint8_t can_ch){
	uint8_t buffer_size = this->getBufferSize(can_ch);

//	if(can_ch == CAN_CH2 && bus2device_bypass_){
//		return 0;
//	}
//	else{
		return buffer_size/2 + buffer_size%2;
//	}
}

int CanTransfer::configTiming(uint32_t can_ch, uint16_t prescaler,
		uint16_t sjw, uint16_t tseg1, uint16_t tseg2,
		uint16_t data_prescaler, uint16_t data_sjw,
		uint16_t data_tseg1, uint16_t data_tseg2,
		bool fd) {
	FDCAN_HandleTypeDef *hfdcan;

	if (can_ch == CAN_CH1) {
		hfdcan = hfdcan1_;
	} else if (can_ch == CAN_CH2) {
		hfdcan = hfdcan2_;
	} else if (can_ch == CAN_CH3) {
		hfdcan = hfdcan3_;
	}
	else {
		return -1;
	}

	HAL_FDCAN_DeInit(hfdcan);
	if (can_ch == CAN_CH1) {
		hfdcan->Instance = FDCAN1;
	}
	if (can_ch == CAN_CH2) {
		hfdcan->Instance = FDCAN2;
	}

	if (fd) {
		hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	} else {
		hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	}
	hfdcan->Init.NominalPrescaler = prescaler;
	hfdcan->Init.NominalSyncJumpWidth = sjw;
	hfdcan->Init.NominalTimeSeg1 = tseg1;
	hfdcan->Init.NominalTimeSeg2 = tseg2;
	hfdcan->Init.DataPrescaler = data_prescaler;
	hfdcan->Init.DataSyncJumpWidth = data_sjw;
	hfdcan->Init.DataTimeSeg1 = data_tseg1;
	hfdcan->Init.DataTimeSeg2 = data_tseg2;

	if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
		return -1;
	}

	return 0;
}

int CanTransfer::busState(uint32_t can_ch, bool *bus_off, bool *error_passive,
		uint8_t *error_code, uint8_t *data_error_code) {
	FDCAN_HandleTypeDef *hfdcan;

	if (can_ch == CAN_CH1) {
		hfdcan = hfdcan1_;
	} else if (can_ch == CAN_CH2) {
		hfdcan = hfdcan2_;
	} else if (can_ch == CAN_CH3) {
		hfdcan = hfdcan3_;
	}
	else {
		return -1;
	}

	FDCAN_ProtocolStatusTypeDef state;
	HAL_FDCAN_GetProtocolStatus(hfdcan, &state);

	if (state.LastErrorCode == FDCAN_PSR_LEC_NO_CHANGE) {
		*error_code = prev_error_code[can_ch];
	} else {
		*error_code = state.LastErrorCode;
		prev_error_code[can_ch] = state.LastErrorCode;
	}

	if (state.DataLastErrorCode == FDCAN_PSR_LEC_NO_CHANGE) {
		*data_error_code = prev_data_error_code[can_ch];
	} else {
		*data_error_code = state.DataLastErrorCode;
		prev_data_error_code[can_ch] = state.DataLastErrorCode;
	}

	*bus_off = state.BusOff;
	*error_passive = state.ErrorPassive;

	return 0;
}

uint8_t CanTransfer::getTxPointer(void) {
	return tx_queue_pointer_;
}

int CanTransfer::registHalfSignal(uint8_t can_ch, osThreadId thread,
		int32_t value) {
	if (value == 0) return -1;

	half_signal_thread[can_ch] = thread;
	half_signal_value[can_ch] = value;

	return 0;
}

void CanTransfer::incrementTimestemp(void) {
	local_timestamp++;
}

bool CanTransfer::getBusErrorPassive(uint8_t can_ch) {
	FDCAN_HandleTypeDef *hfdcan;

	if (can_ch == CAN_CH1) {
		hfdcan = hfdcan1_;
	} else if (can_ch == CAN_CH2) {
		hfdcan = hfdcan2_;
	} else if (can_ch == CAN_CH3) {
		hfdcan = hfdcan3_;
	}
	else {
		return false;
	}

	uint32_t StatusReg = READ_REG(hfdcan->Instance->PSR);

	return ((StatusReg & FDCAN_PSR_EP) >> FDCAN_PSR_EP_Pos) != 0;
}

bool CanTransfer::isBusOn(uint8_t can_ch) {
	return bus_on[can_ch];
}

void CanTransfer::setBus2DeviceBypassMode(const bool& flag){
	bus2device_bypass_ = flag;
}

void CanTransfer::setMsgFilteringMode(const bool& flag){
	filtering_message_ = flag;
}
