/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Node LoRa TDMA (No printf version)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include "sht3x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Định nghĩa các trạng thái hoạt động của Node (Máy trạng thái - State Machine)
typedef enum {
    NODE_STATE_WAIT_BEACON,  // Đang chờ tín hiệu đồng bộ (Beacon) từ Gateway
    NODE_STATE_JOINING,		 // Đang xin gia nhập mạng (chưa có ID)
    NODE_STATE_TDMA_WAIT,	 // Đã đồng bộ, đang chờ đến khe thời gian (Slot) của mình
    NODE_STATE_SEND_DATA,	 // Đang gửi dữ liệu cảm biến đi
    NODE_STATE_SLEEP,		 // Ngủ đông để tiết kiệm pin chờ chu kỳ sau
    NODE_STATE_LOST_SYNC	 // Mất kết nối quá lâu (mất đồng bộ)
} node_state_t;

#define PKT_BEACON    1		 // Gói đồng bộ từ Gateway
#define PKT_DATA      2		 // Gói dữ liệu từ Node gửi lên
#define PKT_JOIN_REQ  3		 // Gói yêu cầu gia nhập từ Node mới
#define PKT_JOIN_ACK  4		 // Gói chấp nhận gia nhập từ Gateway


// --- CẤU TRÚC GÓI TIN (STRUCT) ---
// __attribute__((packed)) giúp nén struct lại, không cho compiler chèn byte trống (padding)
// để đảm bảo kích thước gói tin nhỏ nhất và khớp giữa các thiết bị khác nhau.

typedef struct __attribute__((packed)) {
    uint8_t  type;			 // Loại gói tin (PKT_BEACON = 1)
    uint16_t cycleID;		 // Số thứ tự chu kỳ hiện tại (để phát hiện trùng lặp)
    uint32_t serverTime;	 // Thời gian thực từ Server (Node dùng để chỉnh đồng hồ nếu cần)
    uint16_t cycleDuration;	 // Độ dài chu kỳ mạng (ví dụ 30s)
    uint16_t reportInterval; // Tần suất gửi báo cáo (ví dụ: 1 chu kỳ gửi 1 lần, hay 5 chu kỳ gửi 1 lần)
    uint8_t  checksum;		 // Byte kiểm tra lỗi
} BeaconPacket_t; 			 // Tổng: 10 Bytes

typedef struct __attribute__((packed)) {
    uint8_t  type;			 // Loại gói tin (PKT_DATA = 2)
    uint8_t  node_id; 		 // ID của Node gửi (được cấp bởi Gateway)
    int16_t  temperature;	 // Nhiệt độ nhân 100 (để giữ phần thập phân)
    uint16_t humidity;		 // Độ ẩm nhân 10
    uint8_t  soil_moisture;	 // Độ ẩm đất (%)
    uint8_t  battery_level;	 // Phần trăm pin (%)
    uint8_t  checksum;		 // Byte kiểm tra lỗi
} DataPacket_t; 			 // Tổng: 9 bytes

typedef struct __attribute__((packed)) {
    uint8_t  type;			 // Loại gói tin (PKT_JOIN_REQ = 3)
    uint32_t chipID;		 // Mã định danh cứng (Unique ID) của chip STM32
    uint8_t  checksum;		 // Byte kiểm tra lỗi
} JoinReqPacket_t; 			 // Tổng: 6 Bytes

typedef struct __attribute__((packed)) {
    uint8_t  type;			 // Loại gói tin (PKT_JOIN_ACK = 4)
    uint32_t chipID;		 // Trả lại chipID để Node biết gói này dành cho mình
    uint8_t  assignedID;	 // ID mới mà Gateway cấp cho Node (Slot)
    uint8_t  checksum;	  	 // Byte kiểm tra lỗi
} JoinAckPacket_t; 			 // Tổng: 7 Bytes

#define SLOT_TIME_MS  250				// Mỗi Node có 250ms để gửi
#define BEACON_TX_TIME_MS 100			// Thời gian dành cho Beacon
#define BASE_OFFSET_MS   1500			// Thời gian chờ sau Beacon trước khi bắt đầu Slot 1 (Phase Control)
#define JOIN_PHASE_START_MS  26500		// Thời điểm bắt đầu cho phép gửi Join Request (giây thứ 26.5)
#define FLASH_STORAGE_ADDR 0x0800FC00	// Địa chỉ bộ nhớ Flash dùng để lưu ID (Trang cuối cùng của STM32F103C8)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
sht3x_handle_t sht3x;	  // Biến cấu trúc điều khiển cảm biến SHT3x
LoRa myLoRa;			  // Biến cấu trúc điều khiển module LoRa
uint8_t RxBuffer[128];	  // Bộ đệm chứa dữ liệu nhận được từ LoRa

// --- BIẾN ĐỒNG HỒ HỆ THỐNG  ---
// Biến này cực kỳ quan trọng, được tăng lên 1 mỗi mili-giây bởi ngắt Timer
volatile uint32_t tdma_millis = 0;

// --- CÁC BIẾN CẤU HÌNH ĐỘNG ---
node_state_t current_state;					// Trạng thái hiện tại của Node
uint8_t  current_node_id = 0;				// ID của Node (0 = chưa có ID/Node mới)
uint16_t last_cycleID = 0;					// Lưu ID chu kỳ trước để tránh xử lý trùng Beacon
volatile uint32_t last_beacon_tick = 0;		// Thời điểm (tdma_millis) nhận được Beacon gần nhất
volatile uint32_t cycle_duration_sec = 30;	// Độ dài chu kỳ mặc định là 30s

// Biến quản lý tần suất gửi (Dynamic Reporting)
volatile uint16_t dynamic_report_interval = 1; // Mặc định là gửi mỗi chu kỳ (1)
uint16_t send_cycle_counter = 0;			   // Biến đếm số chu kỳ đã trôi qua

// Biến quản lý lỗi mất kết nối
uint8_t  missed_beacon_count = 0;		// Đếm số lần liên tiếp không thấy Beacon
#define MAX_MISSED_BEACONS 5			// Mất 5 lần liên tiếp coi như mất kết nối

// Cờ ngắt từ phần cứng (volatile vì thay đổi trong hàm ngắt)
extern volatile uint8_t TxDoneFlag;		// Cờ báo gửi xong
extern volatile uint8_t RxDoneFlag; 	// Cờ báo nhận xong

float temp_f, hum_f;			// Nhiệt độ, độ ẩm (float)
uint16_t adc_results[2];		// Kết quả ADC thô (0: Đất, 1: Pin)
uint8_t  soil_pct = 0;			// Độ ẩm đất (%)
uint8_t  bat_pct = 0;			// Pin (%)
uint32_t target_tx_time = 0;	// Thời điểm dự kiến sẽ gửi dữ liệu
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Serial_Printf(const char* format, ...);
void LowPower_Sleep_Millis(uint32_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Hàm ngủ tiết kiệm năng lượng có độ trễ chính xác
void LowPower_Sleep_Millis(uint32_t ms) {
    uint32_t start = tdma_millis;
    // Chờ cho đến khi đủ thời gian trôi qua
    // Phép trừ (tdma_millis - start) xử lý đúng cả khi biến đếm tràn (overflow)
    while (tdma_millis - start < ms) {
    	// Lệnh WFI (Wait For Interrupt): CPU ngừng chạy, giảm tiêu thụ điện
    	// CPU sẽ tỉnh lại khi có ngắt Timer (mỗi 1ms) hoặc ngắt LoRa
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
}

// Hàm đọc và trung bình cộng ADC để giảm nhiễu
void Read_All_ADC(void) {
    uint32_t sum_soil = 0;
    uint32_t sum_bat = 0;
    const uint8_t SAMPLES = 10;		// Lấy mẫu 10 lần

    // Dừng ADC để reset trạng thái
    HAL_ADC_Stop(&hadc1);

    for (int i = 0; i < SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);		// Bắt đầu chuyển đổi
        // Đọc kênh đầu tiên (Độ ẩm đất)
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            sum_soil += HAL_ADC_GetValue(&hadc1);
        }
        // Đọc kênh tiếp theo (Pin - nhờ chế độ Scan của ADC)
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            sum_bat += HAL_ADC_GetValue(&hadc1);
        }
        LowPower_Sleep_Millis(5); 	 // Nghỉ 5ms giữa các lần đo
    }

    HAL_ADC_Stop(&hadc1);

    // Tính trung bình
    adc_results[0] = (uint16_t)(sum_soil / SAMPLES);
    adc_results[1] = (uint16_t)(sum_bat / SAMPLES);
}

// Hàm tính phần trăm pin từ giá trị ADC
uint8_t Calculate_Battery_Percent(uint16_t raw_val) {

	    uint16_t val_max = 1866;
	    uint16_t val_min = 1497;

	    // 1. Cắt ngọn (Clamping) để giá trị không vượt quá giới hạn
	    if (raw_val > val_max) raw_val = val_max;
	    if (raw_val < val_min) raw_val = val_min;

	    // 2. Tính phần trăm tuyến tính
	    uint32_t range = val_max - val_min;

	    // Ép kiểu sang uint32_t để tránh tràn số khi nhân với 100
	    uint8_t pct = (uint8_t)(((uint32_t)(raw_val - val_min) * 100) / range);

	    return pct;
}

// Hàm chuyển đổi ADC sang độ ẩm đất %
float SoilMoisture_Convert(uint16_t adc_value)
{
	// CÁC GIÁ TRỊ NÀY CẦN HIỆU CHỈNH THỰC TẾ (CALIBRATION)
	// Ví dụ: Nhúng vào nước được 1750, để khô được 2040
    if (adc_value > 2040) adc_value = 2040;
    if (adc_value < 1750)  adc_value = 1750;
    // Công thức tính tỷ lệ nghịch (ADC càng nhỏ -> càng ẩm)
    float pct = (float)(2040 - adc_value) * 60.0f / (2040 - 1750);
    return pct;
}

// HÀM LƯU ID VÀO BỘ NHỚ FLASH
void Flash_Write_ID(uint8_t id) {
	// Nếu ID không thay đổi thì không ghi lại (để bảo vệ tuổi thọ Flash)
    if (current_node_id == id) return;

    HAL_FLASH_Unlock();	// Mở khóa Flash
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

    // Cấu hình xóa 1 trang nhớ (Page)
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_STORAGE_ADDR;
    EraseInitStruct.NbPages     = 1;

    // Thực hiện xóa
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
    	// Ghi dữ liệu: Magic Number (0xA5) ở byte cao + ID ở byte thấp
        uint32_t data = (0xA5 << 8) | id;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR, data);
        current_node_id = id;	// Cập nhật biến toàn cục
        Serial_Printf("[FLASH] Saved ID: %d to 0x%X\r\n", id, FLASH_STORAGE_ADDR);
    } else {
        Serial_Printf("[FLASH] Erase Error!\r\n");
    }
    HAL_FLASH_Lock();	// Khóa lại Flash
}

// HÀM ĐỌC ID TỪ FLASH KHI KHỞI ĐỘNG
void Flash_Read_ID(void) {
	// Đọc trực tiếp từ địa chỉ bộ nhớ
    uint32_t data = *(__IO uint32_t *)FLASH_STORAGE_ADDR;
    // Kiểm tra Magic Number (0xA5) xem dữ liệu có hợp lệ không
    if ((data >> 8) == 0xA5) {
        current_node_id = (uint8_t)(data & 0xFF);	// Lấy byte thấp làm ID
        Serial_Printf("[FLASH] LOADED ID: %d\r\n", current_node_id);
    } else {
    	// Flash trắng hoặc lỗi -> Node mới
        current_node_id = 0;
        Serial_Printf("[FLASH] No ID found (New Node)\r\n");
    }
}

// Hàm in ra UART tuỳ chỉnh (giống printf)
void Serial_Printf(const char* format, ...) {
    char buff[64]; 											// Bộ đệm tạm
    va_list args;
    va_start(args, format);
    vsnprintf(buff, sizeof(buff), format, args);			// Format chuỗi
    va_end(args);
    // Gửi qua UART1
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 100);
}

// Lấy mã định danh duy nhất của chip (Unique Device ID)
// Địa chỉ này là cố định cho dòng STM32F103
uint32_t Get_ChipID(void) {
    return *(uint32_t*)(0x1FFFF7E8); // Địa chỉ UID STM32F103
}

// Hàm tính XOR Checksum (đơn giản, nhanh)
uint8_t XOR_Calc(const void *buf, uint8_t payload_len) {
    uint8_t x = 0;
    uint8_t *p = (uint8_t*)buf;
    // XOR tất cả các byte lại với nhau
    for (uint8_t i = 0; i < payload_len; i++) x ^= p[i];
    return x;
}

// Hàm gửi gói dữ liệu (Data Packet) qua LoRa
void LoRa_Send_Data_HW(void) {
	// 1. Đóng gói dữ liệu vào struct
     DataPacket_t data = {
        .type          = PKT_DATA,
		.node_id       = current_node_id,
        .temperature   = (int16_t)(temp_f * 100),
        .humidity      = (uint16_t)(hum_f * 10),
        .soil_moisture = soil_pct,
		.battery_level = bat_pct,
    };
     // 2. Tính checksum
    data.checksum = XOR_Calc(&data, sizeof(DataPacket_t) - 1);
    // 3. Quy trình gửi phần cứng SX1278
    LoRa_gotoMode(&myLoRa, STNBY_MODE);							// Phải về Standby mới ghi được FIFO
    LoRa_write(&myLoRa, RegIrqFlags, 0xFF);						// Xóa cờ ngắt cũ
    uint8_t txBase = LoRa_read(&myLoRa, RegFiFoTxBaseAddr);
    LoRa_write(&myLoRa, RegFiFoAddPtr, txBase);					// Đặt con trỏ ghi

    // Ghi từng byte vào FIFO
    for(int i=0; i<sizeof(data); i++) LoRa_write(&myLoRa, RegFiFo, ((uint8_t*)&data)[i]);
    LoRa_write(&myLoRa, RegPayloadLength, sizeof(data));	    // Thiết lập độ dài
    LoRa_write(&myLoRa, RegDioMapping1, 0x40);					// Cấu hình DIO0 kích hoạt khi TxDone
    LoRa_gotoMode(&myLoRa, TRANSMIT_MODE);						// Kích hoạt chế độ gửi
}

// Hàm gửi yêu cầu gia nhập (Join Request)
void LoRa_Send_JoinReq_XOR(void) {
	// Random delay một chút để tránh xung đột nếu nhiều node bật cùng lúc
	// Dùng ChipID và thời gian hệ thống làm seed ngẫu nhiên
	srand(HAL_GetTick() ^ Get_ChipID());
	uint32_t safe_delay = 50 + (rand() % 200);
	LoRa_gotoMode(&myLoRa, STNBY_MODE);
	LowPower_Sleep_Millis(safe_delay);	// Ngủ chờ random

	// 1. Tạo gói tin
    JoinReqPacket_t req = {
    		.type = PKT_JOIN_REQ,
			.chipID = Get_ChipID()		// Gửi ChipID để Gateway nhận diện
    };
    req.checksum = XOR_Calc(&req, sizeof(req) - 1);

    // 2. Quy trình gửi (tương tự hàm trên)
    LoRa_gotoMode(&myLoRa, STNBY_MODE);
    uint8_t txBase = LoRa_read(&myLoRa, RegFiFoTxBaseAddr);
    LoRa_write(&myLoRa, RegFiFoAddPtr, txBase);
    for(int i=0; i<sizeof(req); i++) LoRa_write(&myLoRa, RegFiFo, ((uint8_t*)&req)[i]);
    LoRa_write(&myLoRa, RegPayloadLength, sizeof(req));
    LoRa_gotoMode(&myLoRa, TRANSMIT_MODE);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // 1. Khởi tạo cảm biến SHT3x
  sht3x.i2c_handle = &hi2c1;
  sht3x.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW;
  if (!sht3x_init(&sht3x)) {
      Serial_Printf("[ERROR] SHT3x Init Failed!\r\n");
  } else {
	  Serial_Printf("[OK] SHT3x Ready.\r\n");
  }

  // 2. Khởi tạo module LoRa SX1278
  myLoRa = newLoRa();
  myLoRa.CS_port = NSS_GPIO_Port; myLoRa.CS_pin = NSS_Pin;
  myLoRa.reset_port = RST_GPIO_Port; myLoRa.reset_pin = RST_Pin;
  myLoRa.DIO0_port = DIO0_GPIO_Port; myLoRa.DIO0_pin = DIO0_Pin;
  myLoRa.hSPIx = &hspi1;

  myLoRa.frequency = 433;				// Tần số 433MHz
  myLoRa.spredingFactor = SF_7;			// SF7 (Tốc độ nhanh, tầm xa trung bình)
  myLoRa.bandWidth = BW_125KHz;			// Băng thông 125kHz
  myLoRa.crcRate = CR_4_5;				// Mã sửa lỗi
  myLoRa.power = POWER_17db;			// Công suất phát cao nhất
  myLoRa.overCurrentProtection = 100;
  myLoRa.preamble = 8;

  LoRa_reset(&myLoRa);		// Reset cứng module
  if (LoRa_init(&myLoRa) == LORA_OK) {
      Serial_Printf("[OK] LoRa Init Success.\r\n");
  } else {
      Serial_Printf("[ERROR] LoRa Init Failed: %d\r\n", LoRa_init(&myLoRa));
  }

  LoRa_setSyncWord(&myLoRa, 0x12);			// Mã đồng bộ mạng (phải giống nhau cả mạng)
  LoRa_setTOMsb_setCRCon(&myLoRa);			// Bật CRC
  LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);	// Chuyển sang chế độ nhận liên tục
  HAL_TIM_Base_Start_IT(&htim1);			// Bật Timer 1 ngắt (để tăng biến tdma_millis)


  Serial_Printf(">>> NODE START | UID: 0x%lX <<<\r\n", Get_ChipID());
  Flash_Read_ID();							// Đọc ID từ bộ nhớ Flash

  current_state = NODE_STATE_WAIT_BEACON;	// Bắt đầu ở trạng thái chờ Beacon
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // --- MÁY TRẠNG THÁI (STATE MACHINE) ---
	  // Chương trình chính là một switch-case lớn xử lý từng trạng thái
	  switch (current_state) {
	  	  	  // --- TRẠNG THÁI 1: CHỜ BEACON ---
	  	  	  case NODE_STATE_WAIT_BEACON: {
	               LoRa_write(&myLoRa, RegDioMapping1, 0x00);	// Đảm bảo LoRa đang ở chế độ nhận
	               LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);

	               uint32_t wait_start = tdma_millis;			// Ghi lại thời gian bắt đầu chờ
	               bool beacon_ok = false;

	               // Vòng lặp chờ tối đa 40 giây (chu kỳ mạng là 30s -> chờ 40s cho chắc)
	               while ((uint32_t)(tdma_millis - wait_start) < 40000) {
	            	      // Kiểm tra cờ ngắt báo có gói tin đến
	                      if (RxDoneFlag) {
	                          RxDoneFlag = 0;		// Xóa cờ
	                          uint8_t len = LoRa_read(&myLoRa, RegRxNbBytes);
	                          // Kiểm tra độ dài có đúng là gói Beacon không
	                          if (len == sizeof(BeaconPacket_t)) {
	                              LoRa_Read_Buffer(&myLoRa, RxBuffer, len);

	                              // Kiểm tra Checksum
	                              if (RxBuffer[len-1] != XOR_Calc(RxBuffer, len - 1)) continue;

	                              BeaconPacket_t *b = (BeaconPacket_t*)RxBuffer;
	                              if (b->type == PKT_BEACON) {
	                                  // Kiểm tra trùng lặp Beacon
	                                  if (b->cycleID == last_cycleID && last_cycleID != 0) continue;
	                                  // --- ĐỒNG BỘ THÀNH CÔNG ---
	                                  last_beacon_tick = tdma_millis;					// Cập nhật mốc thời gian chuẩn
	                                  last_cycleID = b->cycleID;
	                                  cycle_duration_sec = b->cycleDuration;
	                                  dynamic_report_interval = b->reportInterval;		// Cập nhật tần suất báo cáo động từ Gateway
	                                  if(dynamic_report_interval == 0) dynamic_report_interval = 1;
	                                  beacon_ok = true;
	                                  missed_beacon_count = 0;							// Reset đếm lỗi
	                                  Serial_Printf("[SYNC] Cycle %d | Dur:%d | Rep:%d\r\n", last_cycleID, cycle_duration_sec, dynamic_report_interval);

	                                  // --- PHÂN NHÁNH XỬ LÝ SAU BEACON ---
	                                  if (current_node_id != 0) {
	                                	  // --> TRƯỜNG HỢP A: ĐÃ CÓ ID (Node cũ)
	                                	  // Kiểm tra xem chu kỳ này có cần gửi không (Dynamic Reporting)
	                                	  send_cycle_counter++;
	                                	  if (send_cycle_counter >= dynamic_report_interval) {
	                                	      Serial_Printf("-> Time to SEND (%d/%d)\r\n", send_cycle_counter, dynamic_report_interval);
	                                	      send_cycle_counter = 0;
	                                	      current_state = NODE_STATE_TDMA_WAIT;		// Chuyển sang chờ gửi
	                                	  } else {
	                                		  // Chưa đến lượt gửi -> Ngủ tiếp
	                                	      Serial_Printf("-> Sleep & Sync (%d/%d)\r\n", send_cycle_counter, dynamic_report_interval);
	                                	      current_state = NODE_STATE_SLEEP;
	                                	  }
	                                  } else {
	                                      // --> TRƯỜNG HỢP B: CHƯA CÓ ID (Node mới)
	                                      Serial_Printf("[NO ID] Listening for ACK from Gateway...\r\n");
	                                      // Gateway sẽ gửi gói ACK cho các node khác ngay sau Beacon
	                                      // Node mới cần "nghe lỏm" xem có gói ACK nào dành cho mình không (từ yêu cầu join trước đó)
	                                      // Mở cửa sổ nghe trong 3 giây
	                                      uint32_t ack_wait_start = tdma_millis;
	                                      bool received_ack = false;

	                                      while ((uint32_t)(tdma_millis - ack_wait_start) < 3000) {
	                                          if (RxDoneFlag) {
	                                              RxDoneFlag = 0;
	                                              uint8_t l = LoRa_read(&myLoRa, RegRxNbBytes);
	                                              LoRa_Read_Buffer(&myLoRa, RxBuffer, l);			// Đọc buffer để xóa FIFO

	                                              if (l == sizeof(JoinAckPacket_t)) {
	                                                  JoinAckPacket_t *ack = (JoinAckPacket_t*)RxBuffer;
	                                                  // Kiểm tra xem ACK có chứa ChipID của mình không
	                                                  if (ack->type == PKT_JOIN_ACK && ack->chipID == Get_ChipID() && ack->checksum == XOR_Calc(ack, l - 1))
	                                                  {
	                                                	  // ĐƯỢC CHẤP NHẬN! Lưu ID vào Flash
	                                                      Flash_Write_ID(ack->assignedID);
	                                                      Serial_Printf("[JOIN] SUCCESS via ACK! ID=%d\r\n", current_node_id);
	                                                      received_ack = true;
	                                                      break; // Thoát vòng lặp chờ ACK
	                                                  }
	                                              }
	                                          }
	                                          // Ngủ nhẹ trong lúc chờ ACK (CPU sleep, LoRa vẫn on)
	                                          HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	                                      }

	                                      if (received_ack) {
	                                    	  // Nếu nhận được ACK -> Có ID rồi -> Vào chờ gửi Data
	                                          current_state = NODE_STATE_TDMA_WAIT;
	                                      } else {
	                                    	  // Nghe 3s mà không thấy ai gọi tên mình -> Đi gửi đơn xin gia nhậpSerial_Printf("[TIMEOUT] No ACK received. Requesting Join...\r\n");
	                                          current_state = NODE_STATE_JOINING;
	                                      }
	                                  }
	                                  break;  // Thoát vòng lặp chờ Beacon (vì đã nhận được rồi)
	                              }
	                          }
	                      }
	                      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	               	  }
	                  // Nếu hết 40s mà không thấy Beacon -> Mất đồng bộ
	                  if (!beacon_ok) {
	                      missed_beacon_count++;
	                      Serial_Printf("[WARN] Miss Beacon (%d)\r\n", missed_beacon_count);
	                      if (missed_beacon_count >= MAX_MISSED_BEACONS) current_state = NODE_STATE_LOST_SYNC;
	                      else current_state = NODE_STATE_SLEEP;	// Ngủ tạm chờ chu kỳ sau
	                  }
	                  break;
	            }

	  	  	  	// --- TRẠNG THÁI 2: XIN GIA NHẬP ---
	            case NODE_STATE_JOINING: {
	            	  // Nếu ID rồi thì bỏ qua bước này
	                  if (current_node_id != 0) {
	                	  current_state = NODE_STATE_TDMA_WAIT;
	                	  break;
	                  }
	                  // Tính thời điểm bắt đầu Phase Join (theo thiết kế là giây thứ 26.5 sau Beacon)
	                  uint32_t join_target = last_beacon_tick + JOIN_PHASE_START_MS;

	                  // Nếu chưa đến giờ, ngủ chờ tiết kiệm pin
	                  if ((int32_t)(join_target - tdma_millis) > 0) {
	                       uint32_t sleep_time = join_target - tdma_millis;
	                       Serial_Printf("[JOIN] Too early. Sleep %lu ms until Phase 3...\r\n", sleep_time);
	                       LowPower_Sleep_Millis(sleep_time);
	                  }
	                  // Gửi gói Join Request (có random delay bên trong hàm)
	                  LoRa_Send_JoinReq_XOR();
	                  Serial_Printf("-> Sending JoinReq...\r\n");

	                  // Chờ cho đến khi gửi xong (TxDone)
	                  uint32_t t0 = tdma_millis;
	                  while (!TxDoneFlag && (uint32_t)(tdma_millis - t0) < 1500) {
	                       // Busy wait để chắc chắn không miss IRQ
	                  }
	                  TxDoneFlag = 0;   // Xóa cờ Tx
	                  RxDoneFlag = 0;   // Xóa cờ Rx
	                  // Chuyển ngay về trạng thái chờ Beacon để đón chu kỳ mới.
	                  Serial_Printf("[JOIN] Req sent. Waiting for Beacon & ACK next cycle...\r\n");
	                  current_state = NODE_STATE_WAIT_BEACON;
	                  break;
	            }

	            case NODE_STATE_TDMA_WAIT: {
	                 uint8_t my_slot = current_node_id - 1;
	                 target_tx_time = last_beacon_tick + BASE_OFFSET_MS + (uint32_t)my_slot * SLOT_TIME_MS;

	                 // Còn bao lâu nữa đến giờ G?
	                 int32_t remaining = (int32_t)target_tx_time - (int32_t)tdma_millis;

	                 if (remaining > 0) {
	                 	 // Còn sớm -> Ngủ đúng bằng thời gian còn lại
	                 	 LowPower_Sleep_Millis(remaining);
	                 }
	                 // Tỉnh dậy là đúng giờ G (hoặc lố 1-2ms do sai số, không sao)
	                 current_state = NODE_STATE_SEND_DATA;
	                 break;
	             }

	             // --- TRẠNG THÁI 4: GỬI DỮ LIỆU ---
	             case NODE_STATE_SEND_DATA: {
	            	  // Tính độ lệch thời gian thực tế so với dự kiến
	                  int32_t diff = (int32_t)tdma_millis - (int32_t)target_tx_time;
	                  // Nếu lệch quá +/- 150ms (Guard time) thì không gửi để tránh làm nhiễu Node khác
	                  if (diff < -150 || diff > 150) {
	                      Serial_Printf("[SKIP] Bad Timing (%ld)\r\n", diff);
	                      current_state = NODE_STATE_SLEEP;
	                      break;
	                  }

	                  // 1. Bật nguồn cảm biến (MOSFET kích)
	                  HAL_GPIO_WritePin(SENSOR_PWR_GPIO_Port, SENSOR_PWR_Pin, GPIO_PIN_RESET);
	                  LowPower_Sleep_Millis(20);	// Chờ điện áp ổn định
	                  // 2. Đọc cảm biến
	                  Read_All_ADC();
	                  Serial_Printf("ADC Soil=%d | Batt=%d\r\n", adc_results[0], adc_results[1]);
	                  // 3. Chuyển đổi dữ liệu
	                  soil_pct = (uint8_t)SoilMoisture_Convert(adc_results[0]);
	                  bat_pct  = (uint8_t)Calculate_Battery_Percent(adc_results[1]);
	                  sht3x_read_temperature_and_humidity(&sht3x, &temp_f, &hum_f);
	                  // 4. Tắt nguồn cảm biến (Tiết kiệm điện)
	                  HAL_GPIO_WritePin(SENSOR_PWR_GPIO_Port, SENSOR_PWR_Pin, GPIO_PIN_SET);
	                  // 5. Gửi dữ liệu qua LoRa
	                  LoRa_Send_Data_HW();
	                  Serial_Printf("-> TX Data (Slot %d)\r\n", current_node_id);
	                  // 6. Chờ gửi xong
	                  uint32_t t_tx = tdma_millis;
	                  while (!TxDoneFlag && (uint32_t)(tdma_millis - t_tx) < 100) {
	                       HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	                  }

	                  // Nếu treo (không thấy cờ TxDone) -> Reset LoRa
	                  if (!TxDoneFlag) {
	                       LoRa_reset(&myLoRa);
	                       LoRa_init(&myLoRa);
	                       LoRa_setSyncWord(&myLoRa, 0x12);
	                       Serial_Printf("[ERR] TX Timeout\r\n");
	                  }
	                  TxDoneFlag = 0;
	                  missed_beacon_count = 0;				// Gửi thành công nghĩa là vẫn đồng bộ tốt
	                  current_state = NODE_STATE_SLEEP;		// Xong việc, đi ngủ
	                  break;
	             }

	             // --- TRẠNG THÁI 5: NGỦ ĐÔNG ---
	             case NODE_STATE_SLEEP: {
	                  LoRa_gotoMode(&myLoRa, SLEEP_MODE);	// Cho module LoRa ngủ

	                  // Tính thời điểm Beacon tiếp theo
	                  uint32_t total_cycle_ms = (uint32_t)cycle_duration_sec * 1000;
	                  uint32_t next_beacon = last_beacon_tick + total_cycle_ms;

	                  // Dự kiến dậy sớm 3 giây trước Beacon để đón lõng
	                  uint32_t wake_time = next_beacon - 3000;

	                  if ((int32_t)(wake_time - tdma_millis) > 0) {
	                      LowPower_Sleep_Millis(wake_time - tdma_millis); 	// Ngủ dài
	                  }
	                  current_state = NODE_STATE_WAIT_BEACON;				// Tỉnh dậy, chuyển sang chờ Beacon
	                  break;
	             }

	             // --- TRẠNG THÁI 6: MẤT KẾT NỐI ---
	             case NODE_STATE_LOST_SYNC: {
	                  Serial_Printf("[ERR] LOST SYNC! Waiting...\r\n");
	                  missed_beacon_count = 0;
	                  // Về cơ bản quay lại chờ Beacon.
	                  current_state = NODE_STATE_WAIT_BEACON;
	                  break;
	             }
	        }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Callback ngắt GPIO (khi chân DIO0 của LoRa nhảy mức logic)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Đọc thanh ghi cờ ngắt của SX1278
    if (GPIO_Pin == myLoRa.DIO0_pin)
    {
    	uint8_t irqFlags = LoRa_read(&myLoRa, RegIrqFlags);

    	    if (irqFlags & 0x08) TxDoneFlag = 1; // TxDone
    	    if (irqFlags & 0x40) RxDoneFlag = 1; // RxDone

    	    // Xóa cờ ngắt phần cứng ngay lập tức
    	    LoRa_write(&myLoRa, RegIrqFlags, 0xFF);
    }
}

// Callback ngắt Timer (xảy ra mỗi khi Timer tràn - cấu hình 1ms)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        tdma_millis++; // Tăng biến đếm thời gian hệ thống
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
