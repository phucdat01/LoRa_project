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
typedef enum {
    NODE_STATE_WAIT_BEACON,
    NODE_STATE_JOINING,
    NODE_STATE_TDMA_WAIT,
    NODE_STATE_SEND_DATA,
    NODE_STATE_SLEEP,
    NODE_STATE_LOST_SYNC
} node_state_t;

#define PKT_BEACON    1
#define PKT_DATA      2
#define PKT_JOIN_REQ  3
#define PKT_JOIN_ACK  4


// --- CẤU TRÚC GÓI TIN ---

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint16_t cycleID;
    uint32_t serverTime;
    uint16_t cycleDuration;
    uint16_t reportInterval;
    uint8_t  checksum;
} BeaconPacket_t; // 10 Bytes

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  node_id;
    int16_t  temperature;
    uint16_t humidity;
    uint8_t  soil_moisture;
    uint8_t  battery_level;
    uint8_t  checksum;
} DataPacket_t; // 9 bytes

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint32_t chipID;
    uint8_t  checksum;
} JoinReqPacket_t; // 6 Bytes

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint32_t chipID;
    uint8_t  assignedID;
    uint8_t  checksum;
} JoinAckPacket_t; // 7 Bytes

#define SLOT_TIME_MS  250
#define BEACON_TX_TIME_MS 100
#define BASE_OFFSET_MS   1500
#define JOIN_PHASE_START_MS  26500
#define FLASH_STORAGE_ADDR 0x0800FC00
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
sht3x_handle_t sht3x;
LoRa myLoRa;
uint16_t LoRa_status;
uint8_t RxBuffer[128];

// --- BIẾN ĐỒNG HỒ HỆ THỐNG  ---
volatile uint32_t tdma_millis = 0;

// --- CÁC BIẾN CẤU HÌNH ĐỘNG ---
node_state_t current_state;
uint8_t  current_node_id = 0;
uint16_t last_cycleID = 0;
volatile uint32_t last_beacon_tick = 0;
volatile uint32_t cycle_duration_sec = 30;

// [NEW] Biến quản lý tần suất gửi (Dynamic Reporting)
volatile uint16_t dynamic_report_interval = 1; // Mặc định 1 (Gửi liên tục)
uint16_t send_cycle_counter = 0;

// Biến quản lý lỗi
uint8_t  missed_beacon_count = 0;
#define MAX_MISSED_BEACONS 5

// Cờ ngắt
extern volatile uint8_t TxDoneFlag;
extern volatile uint8_t RxDoneFlag;

float temp_f, hum_f;
uint16_t adc_results[2];
uint8_t  soil_pct = 0;
uint8_t  bat_pct = 0;
uint32_t target_tx_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Serial_Printf(const char* format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Read_All_ADC(void) {
    uint32_t sum_soil = 0;
    uint32_t sum_bat = 0;
    const uint8_t SAMPLES = 10;

    // Reset ADC trước khi đo
    HAL_ADC_Stop(&hadc1);

    for (int i = 0; i < SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);

        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            sum_soil += HAL_ADC_GetValue(&hadc1);
        }
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            sum_bat += HAL_ADC_GetValue(&hadc1);
        }
        LowPower_Sleep_Millis(5);
    }

    HAL_ADC_Stop(&hadc1);

    // Tính trung bình
    adc_results[0] = (uint16_t)(sum_soil / SAMPLES);
    adc_results[1] = (uint16_t)(sum_bat / SAMPLES);
}

uint8_t Calculate_Battery_Percent(uint16_t raw_val) {
	    uint16_t val_max = 1730;
	    uint16_t val_min = 1340;

	    // 1. Cắt ngọn (Clamping)
	    if (raw_val > val_max) raw_val = val_max;
	    if (raw_val < val_min) raw_val = val_min;

	    // 2. Tính phần trăm tuyến tính
	    // Khoảng đo: 2260 - 1750 = 510 đơn vị
	    uint32_t range = val_max - val_min;

	    // Ép kiểu sang uint32_t khi nhân để tránh tràn số (overflow) trước khi chia
	    uint8_t pct = (uint8_t)(((uint32_t)(raw_val - val_min) * 100) / range);

	    return pct;
}

void LowPower_Sleep_Millis(uint32_t ms) {
    uint32_t start = tdma_millis;
    // Sử dụng phép trừ số nguyên không dấu để an toàn khi overflow
    while (tdma_millis - start < ms) {
        // CPU dừng thực thi, chờ ngắt (Timer 1 hoặc LoRa) để thức dậy kiểm tra điều kiện while
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
}

float SoilMoisture_Convert(uint16_t adc_value)
{
    // HIỆU CHỈNH THỰC TẾ Ở ĐÂY
    if (adc_value > 2500) adc_value = 2500;
    if (adc_value < 830)  adc_value = 830;
    float pct = (float)(2500 - adc_value) * 60.0f / (2500 - 830);
    return pct;
}
// HÀM LƯU ID VÀO FLASH (Chuẩn)
void Flash_Write_ID(uint8_t id) {
    // Chỉ ghi nếu ID khác giá trị hiện tại để bảo vệ tuổi thọ Flash
    if (current_node_id == id) return;

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

    // Xóa page trước
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_STORAGE_ADDR;
    EraseInitStruct.NbPages     = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        // Ghi Magic Number (0xA5) + ID
        uint32_t data = (0xA5 << 8) | id;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR, data);
        current_node_id = id;
        Serial_Printf("[FLASH] Saved ID: %d to 0x%X\r\n", id, FLASH_STORAGE_ADDR);
    } else {
        Serial_Printf("[FLASH] Erase Error!\r\n");
    }
    HAL_FLASH_Lock();
}

// HÀM ĐỌC ID KHI KHỞI ĐỘNG
void Flash_Read_ID(void) {
    uint32_t data = *(__IO uint32_t *)FLASH_STORAGE_ADDR;
    // Kiểm tra Magic Number
    if ((data >> 8) == 0xA5) {
        current_node_id = (uint8_t)(data & 0xFF);
        Serial_Printf("[FLASH] LOADED ID: %d\r\n", current_node_id);
    } else {
        current_node_id = 0;
        Serial_Printf("[FLASH] No ID found (New Node)\r\n");
    }
}

void Serial_Printf(const char* format, ...) {
    char buff[64]; // Vùng nhớ cục bộ cho mỗi lần gọi
    va_list args;
    va_start(args, format);
    vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 100);
}

uint32_t Get_ChipID(void) {
    return *(uint32_t*)(0x1FFFF7E8); // Địa chỉ UID STM32F103
}

uint8_t XOR_Calc(const void *buf, uint8_t payload_len) {
    uint8_t x = 0;
    uint8_t *p = (uint8_t*)buf;
    for (uint8_t i = 0; i < payload_len; i++) x ^= p[i];
    return x;
}

void LoRa_Send_Data_HW(void) {
     DataPacket_t data = {
        .type          = PKT_DATA,
		.node_id       = current_node_id,
        .temperature   = (int16_t)(temp_f * 100),
        .humidity      = (uint16_t)(hum_f * 10),
        .soil_moisture = soil_pct,
		.battery_level = bat_pct,
    };
    data.checksum = XOR_Calc(&data, sizeof(DataPacket_t) - 1);

    LoRa_gotoMode(&myLoRa, STNBY_MODE);
    LoRa_write(&myLoRa, RegIrqFlags, 0xFF);
    uint8_t txBase = LoRa_read(&myLoRa, RegFiFoTxBaseAddr);
    LoRa_write(&myLoRa, RegFiFoAddPtr, txBase);
    for(int i=0; i<sizeof(data); i++) LoRa_write(&myLoRa, RegFiFo, ((uint8_t*)&data)[i]);
    LoRa_write(&myLoRa, RegPayloadLength, sizeof(data));
    LoRa_write(&myLoRa, RegDioMapping1, 0x40);
    LoRa_gotoMode(&myLoRa, TRANSMIT_MODE);
}

void LoRa_Send_JoinReq_XOR(void) {
	// SỬA: Seed bằng Tick hệ thống để mỗi lần chạy sẽ khác nhau một chút
	srand(HAL_GetTick() ^ Get_ChipID());
	uint32_t safe_delay = 200 + (rand() % 500);
	LoRa_gotoMode(&myLoRa, STNBY_MODE);
	LowPower_Sleep_Millis(safe_delay);

    JoinReqPacket_t req = { .type = PKT_JOIN_REQ, .chipID = Get_ChipID() };
    req.checksum = XOR_Calc(&req, sizeof(req) - 1);

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
  // 1. Init SHT3x
  sht3x.i2c_handle = &hi2c1;
  sht3x.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW;
  if (!sht3x_init(&sht3x)) {
      Serial_Printf("[ERROR] SHT3x Init Failed!\r\n");
  } else {
	  Serial_Printf("[OK] SHT3x Ready.\r\n");
  }

  // 2. Init LoRa
  myLoRa = newLoRa();
  myLoRa.CS_port = NSS_GPIO_Port; myLoRa.CS_pin = NSS_Pin;
  myLoRa.reset_port = RST_GPIO_Port; myLoRa.reset_pin = RST_Pin;
  myLoRa.DIO0_port = DIO0_GPIO_Port; myLoRa.DIO0_pin = DIO0_Pin;
  myLoRa.hSPIx = &hspi1;

  myLoRa.frequency = 433;
  myLoRa.spredingFactor = SF_7;
  myLoRa.bandWidth = BW_125KHz;
  myLoRa.crcRate = CR_4_5;
  myLoRa.power = POWER_11db;
  myLoRa.overCurrentProtection = 100;
  myLoRa.preamble = 8;

  LoRa_reset(&myLoRa);
  if (LoRa_init(&myLoRa) == LORA_OK) {
      Serial_Printf("[OK] LoRa Init Success.\r\n");
  } else {
      Serial_Printf("[ERROR] LoRa Init Failed: %d\r\n", LoRa_status);
  }

  LoRa_setSyncWord(&myLoRa, 0x12);
  LoRa_setTOMsb_setCRCon(&myLoRa);
  LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);
  HAL_TIM_Base_Start_IT(&htim1);


  Serial_Printf(">>> NODE START | UID: 0x%lX <<<\r\n", Get_ChipID());
  Flash_Read_ID();

  // Nếu đã có ID, vào thẳng trạng thái chờ Beacon
  current_state = NODE_STATE_WAIT_BEACON;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // --- MÁY TRẠNG THÁI ---
	  switch (current_state) {
	  	  	  case NODE_STATE_WAIT_BEACON: {
	               LoRa_write(&myLoRa, RegDioMapping1, 0x00);
	               LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);

	               uint32_t wait_start = tdma_millis;
	               bool beacon_ok = false;

	               // 1. VÒNG LẶP CHỜ BEACON (40s)
	               while ((uint32_t)(tdma_millis - wait_start) < 40000) {
	                      if (RxDoneFlag) {
	                          RxDoneFlag = 0;
	                          uint8_t len = LoRa_read(&myLoRa, RegRxNbBytes);
	                          if (len == sizeof(BeaconPacket_t)) {
	                              LoRa_Read_Buffer(&myLoRa, RxBuffer, len);

	                              // Check Checksum
	                              if (RxBuffer[len-1] != XOR_Calc(RxBuffer, len - 1)) continue;

	                              BeaconPacket_t *b = (BeaconPacket_t*)RxBuffer;
	                              if (b->type == PKT_BEACON) {
	                                  // Kiểm tra trùng lặp Beacon
	                                  if (b->cycleID == last_cycleID && last_cycleID != 0) continue;

	                                  last_beacon_tick = tdma_millis;
	                                  last_cycleID = b->cycleID;
	                                  cycle_duration_sec = b->cycleDuration;
	                                  dynamic_report_interval = b->reportInterval;
	                                  if(dynamic_report_interval == 0) dynamic_report_interval = 1;
	                                  beacon_ok = true;
	                                  missed_beacon_count = 0;
	                                  Serial_Printf("[SYNC] Cycle %d | Dur:%d | Rep:%d\r\n", last_cycleID, cycle_duration_sec, dynamic_report_interval);

	                                  // --- [LOGIC SỬA LỖI VÒNG LẶP JOIN] ---
	                                  if (current_node_id != 0) {
	                                	  // --- LOGIC GỬI ĐỊNH KỲ (TIẾT KIỆM PIN) ---
	                                	  send_cycle_counter++;
	                                	  if (send_cycle_counter >= dynamic_report_interval) {
	                                	      Serial_Printf("-> Time to SEND (%d/%d)\r\n", send_cycle_counter, dynamic_report_interval);
	                                	      send_cycle_counter = 0;
	                                	      current_state = NODE_STATE_TDMA_WAIT;
	                                	  } else {
	                                	      Serial_Printf("-> Sleep & Sync (%d/%d)\r\n", send_cycle_counter, dynamic_report_interval);
	                                	      current_state = NODE_STATE_SLEEP;
	                                	  }
	                                  } else {
	                                      // CHƯA CÓ ID -> KÍCH HOẠT CHẾ ĐỘ "NGHE TRƯỚC"
	                                      Serial_Printf("[NO ID] Listening for ACK from Gateway...\r\n");

	                                      // Gateway sẽ xả ACK ngay sau Beacon. Ta phải nghe ngay lập tức!
	                                      // Nghe trong 3000ms (3 giây)
	                                      uint32_t ack_wait_start = tdma_millis;
	                                      bool received_ack = false;

	                                      while ((uint32_t)(tdma_millis - ack_wait_start) < 3000) {
	                                          if (RxDoneFlag) {
	                                              RxDoneFlag = 0;
	                                              uint8_t l = LoRa_read(&myLoRa, RegRxNbBytes);
	                                              // Đọc buffer dù đúng hay sai để clear FIFO
	                                              LoRa_Read_Buffer(&myLoRa, RxBuffer, l);

	                                              if (l == sizeof(JoinAckPacket_t)) {
	                                                  JoinAckPacket_t *ack = (JoinAckPacket_t*)RxBuffer;
	                                                  // Kiểm tra kỹ xem phải ACK cho mình không
	                                                  if (ack->type == PKT_JOIN_ACK &&
	                                                      ack->chipID == Get_ChipID() &&
	                                                      ack->checksum == XOR_Calc(ack, l - 1))
	                                                  {
	                                                      Flash_Write_ID(ack->assignedID);
	                                                      Serial_Printf("[JOIN] SUCCESS via ACK! ID=%d\r\n", current_node_id);
	                                                      received_ack = true;
	                                                      break; // Thoát vòng lặp chờ ACK
	                                                  }
	                                              }
	                                          }
	                                          // Ngủ nhẹ trong lúc chờ ACK
	                                          HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	                                      }

	                                      if (received_ack) {
	                                          // Nếu nhận được ACK -> Có ID rồi -> Vào chờ gửi Data
	                                          current_state = NODE_STATE_TDMA_WAIT;
	                                      } else {
	                                          // Nghe 3s mà không thấy ai gọi tên mình -> Lúc này mới đi gửi đơn xin
	                                          Serial_Printf("[TIMEOUT] No ACK received. Requesting Join...\r\n");
	                                          current_state = NODE_STATE_JOINING;
	                                      }
	                                  }
	                                  break; // Thoát vòng lặp chờ Beacon (đã xử lý xong Beacon)
	                              }
	                          }
	                      }
	                      HAL_Delay(1);
	                  }

	                  if (!beacon_ok) {
	                      missed_beacon_count++;
	                      Serial_Printf("[WARN] Miss Beacon (%d)\r\n", missed_beacon_count);
	                      if (missed_beacon_count >= MAX_MISSED_BEACONS) current_state = NODE_STATE_LOST_SYNC;
	                      else current_state = NODE_STATE_SLEEP;
	                  }
	                  break;
	            }

	            case NODE_STATE_JOINING: {
	                  if (current_node_id != 0) { current_state = NODE_STATE_TDMA_WAIT; break; }
	                  // 1. Tính thời điểm cửa sổ JOIN mở (Ví dụ: Giây thứ 26.5)
	                  // Công thức: 1500ms (Phase 1) + 100 node * 250ms (Phase 2) = 26500ms

	                  uint32_t join_target = last_beacon_tick + JOIN_PHASE_START_MS;

	                  // 2. Nếu chưa đến giờ mở cửa sổ Join, hãy ngủ chờ
	                  if ((int32_t)(join_target - tdma_millis) > 0) {
	                       uint32_t sleep_time = join_target - tdma_millis;
	                       Serial_Printf("[JOIN] Too early. Sleep %lu ms until Phase 3...\r\n", sleep_time);
	                       LowPower_Sleep_Millis(sleep_time);
	                  }
	                  LoRa_Send_JoinReq_XOR();
	                  Serial_Printf("-> Sending JoinReq...\r\n");

	                  // [FIX 5] CHỜ TX DONE RỒI MỚI CHUYỂN MODE
	                  uint32_t t0 = tdma_millis;
	                  while (!TxDoneFlag && (uint32_t)(tdma_millis - t0) < 1500) {
	                       // Busy wait để chắc chắn không miss IRQ
	                  }
	                  TxDoneFlag = 0; // Clear cờ
	                  RxDoneFlag = 0;
	                  // Chuyển sang RX chờ ACK
	                  LoRa_write(&myLoRa, RegIrqFlags, 0xFF);
	                  LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);

	                  uint32_t js = tdma_millis;
	                  while ((uint32_t)(tdma_millis - js) < 4000) {
	                      if (RxDoneFlag) {
	                           RxDoneFlag = 0;
	                           uint8_t len = LoRa_read(&myLoRa, RegRxNbBytes);
	                           LoRa_Read_Buffer(&myLoRa, RxBuffer, len);

	                           if (len == sizeof(JoinAckPacket_t)) {
	                               JoinAckPacket_t *ack = (JoinAckPacket_t*)RxBuffer;

	                               // [FIX 2] KIỂM TRA CHECKSUM VÀ ID
	                               if (ack->type == PKT_JOIN_ACK && ack->chipID == Get_ChipID() && ack->checksum == XOR_Calc(ack, len - 1)) {
	                                   Flash_Write_ID(ack->assignedID);
	                                   Serial_Printf("[JOIN] OK! ID=%d\r\n", current_node_id);
	                                   current_state = NODE_STATE_WAIT_BEACON;
	                                   break;
	                               }
	                           }
	                      }
	                      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	                  }
	                  // Hết giờ mà không thấy ACK thì quay lại WAIT_BEACON để thử lại vào cycle sau
	                  if (current_state == NODE_STATE_JOINING) current_state = NODE_STATE_WAIT_BEACON;
	                  break;
	            }

	            case NODE_STATE_TDMA_WAIT: {
	                 uint8_t my_slot = current_node_id - 1;
	                 target_tx_time = last_beacon_tick + BASE_OFFSET_MS + (uint32_t)my_slot * SLOT_TIME_MS;

	                 if ((int32_t)(target_tx_time - tdma_millis) > 0) {
	                     uint32_t diff = target_tx_time - tdma_millis;
	                     // Dậy sớm 50ms để chuẩn bị cảm biến
	                     if (diff > 50) LowPower_Sleep_Millis(diff - 50);
	                     current_state = NODE_STATE_SEND_DATA;
	                 } else {
	                     // Đã lố giờ (do xử lý gì đó quá lâu) -> Bỏ qua
	                     current_state = NODE_STATE_SLEEP;
	                 }
	                 break;
	             }

	             case NODE_STATE_SEND_DATA: {
	                  int32_t diff = (int32_t)tdma_millis - (int32_t)target_tx_time;
	                  // [FIX] Cửa sổ chấp nhận sai số ±150ms
	                  if (diff < -150 || diff > 150) {
	                      Serial_Printf("[SKIP] Bad Timing (%ld)\r\n", diff);
	                      current_state = NODE_STATE_SLEEP;
	                      break;
	                  }

	                  // Bật nguồn cảm biến
	                  HAL_GPIO_WritePin(SENSOR_PWR_GPIO_Port, SENSOR_PWR_Pin, GPIO_PIN_RESET);
	                  LowPower_Sleep_Millis(20);
	                  Read_All_ADC();
	                  Serial_Printf("ADC Soil=%d | Batt=%d\r\n", adc_results[0], adc_results[1]);

	                  soil_pct = (uint8_t)SoilMoisture_Convert(adc_results[0]); // Rank 1: Soil
	                  bat_pct  = (uint8_t)Calculate_Battery_Percent(adc_results[1]); // Rank 2: Battery
	                  sht3x_read_temperature_and_humidity(&sht3x, &temp_f, &hum_f);
	                  HAL_GPIO_WritePin(SENSOR_PWR_GPIO_Port, SENSOR_PWR_Pin, GPIO_PIN_SET);

	                  LoRa_Send_Data_HW();
	                  Serial_Printf("-> TX Data (Slot %d)\r\n", current_node_id);

	                  uint32_t t_tx = tdma_millis;
	                  while (!TxDoneFlag && (uint32_t)(tdma_millis - t_tx) < 1500) {
	                       HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	                  }

	                  if (!TxDoneFlag) {
	                       LoRa_reset(&myLoRa);
	                       LoRa_init(&myLoRa);
	                       LoRa_setSyncWord(&myLoRa, 0x12);
	                       Serial_Printf("[ERR] TX Timeout\r\n");
	                  }
	                  TxDoneFlag = 0;
	                  missed_beacon_count = 0;
	                  current_state = NODE_STATE_SLEEP;
	                  break;
	             }

	             case NODE_STATE_SLEEP: {
	                  LoRa_gotoMode(&myLoRa, SLEEP_MODE);
	                  uint32_t total_cycle_ms = (uint32_t)cycle_duration_sec * 1000;
	                  uint32_t next_beacon = last_beacon_tick + total_cycle_ms;
	                  // Dậy sớm 5s để đón Beacon
	                  uint32_t wake_time = next_beacon - 5000;

	                  if ((int32_t)(wake_time - tdma_millis) > 0) {
	                      LowPower_Sleep_Millis(wake_time - tdma_millis);
	                  }
	                  current_state = NODE_STATE_WAIT_BEACON;
	                  break;
	             }

	             case NODE_STATE_LOST_SYNC: {
	                  Serial_Printf("[ERR] LOST SYNC! Waiting...\r\n");
	                  missed_beacon_count = 0;
	                  // [FIX 6] KHÔNG RESET ID - Chỉ quay lại tìm Beacon
	                  // current_node_id = 0; <--- ĐÃ BỎ DÒNG NÀY
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Kiểm tra đúng chân DIO0 của LoRa
    if (GPIO_Pin == myLoRa.DIO0_pin)
    {
    	uint8_t irqFlags = LoRa_read(&myLoRa, RegIrqFlags);

    	    if (irqFlags & 0x08) TxDoneFlag = 1; // TxDone
    	    if (irqFlags & 0x40) RxDoneFlag = 1; // RxDone

    	    // Xóa cờ ngắt phần cứng ngay lập tức
    	    LoRa_write(&myLoRa, RegIrqFlags, 0xFF);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        tdma_millis++; // "Trái tim" của hệ thống, không bao giờ dừng
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
