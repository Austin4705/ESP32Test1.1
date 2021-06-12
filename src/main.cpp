void loop()
{

}

#include <Arduino.h>
// #include "I2SSampler.h"
#include "BluetoothSerial.h"
#include <driver/i2s.h>
#include "esp_adc_cal.h"

BluetoothSerial SerialBT;

// calibration values for the adc
#define DEFAULT_VREF 1100
esp_adc_cal_characteristics_t *adc_chars;

void initBT();
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void startRecording();
void finishRecording();
void i2s_adc(void *arg);
//void i2sInit();
//void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len);
void i2s_cancel(void *arg);
void writerTask(void *param);


char* i2s_read_buff;
uint8_t* flash_write_buff;
TaskHandle_t taskHandler = NULL;

void setup() {
  Serial.begin(115200);

  //Range 0-4096
  adc1_config_width(ADC_WIDTH_BIT_12);
  // full voltage range
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

  // check to see what calibration is available
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
  {
    Serial.println("Using voltage ref stored in eFuse");
  }
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
  {
    Serial.println("Using two point values from eFuse");
  }
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_DEFAULT_VREF) == ESP_OK)
  {
    Serial.println("Using default VREF");
  }
  //Characterize ADC
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  initBT();
  }

void initBT(){
  if(!SerialBT.begin("ESP32-Recorder")){
    Serial.println("An error occurred initalizing Bluetooth");
    ESP.restart();
    return;
  } else{
    Serial.println("Bluetooth Initialized");
  }

  SerialBT.register_callback(btCallback);
  Serial.println("==========================================================");
  Serial.println("The device started, now you can pair it with bluetooth-001");
  Serial.println("==========================================================");
}

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected!");
  }
  else if(event == ESP_SPP_DATA_IND_EVT){
    Serial.printf("ESP_SPP_DATA_IND_EVT len=%d handle=%d\n", param->data_ind.len, param->data_ind.handle);
    int dataLen = param->data_ind.len + 1;
    char textArray[dataLen];
    strncpy(textArray, (const char*)param->data_ind.data, dataLen);
    textArray[dataLen - 1] = 0;
    String textString = textArray;
    Serial.print("*** Text String: ");
    Serial.print(textString);
    Serial.println("\n");

    if(textString.equals("START")){
      startRecording();
    }else if(textString.equals("STOP")){
      finishRecording();
    }
  }
}

void startRecording(){
  Serial.println(" *** Recording Start ***");
  // i2s_read_buff = (char*) calloc(I2S_READ_LEN, sizeof(char));
  // flash_write_buff = (uint8_t*) calloc(I2S_READ_LEN, sizeof(char));
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2, NULL, 1, &taskHandler);
}

void finishRecording(){
  Serial.println(" *** Recording End ***");
  xTaskCreate(i2s_cancel, "i2s_cancel", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void i2s_cancel(void *arg){
  vTaskDelay(2000);
  Serial.println(" *** Recording End 2 *** ");
  // if(i2s_read_buff != NULL){
  //   free(i2s_read_buff);
  //   i2s_read_buff = NULL;
  // }

  // if(flash_write_buff != NULL){
  //  free(flash_write_buff);
  //  flash_write_buff = NULL; 
  // }

  vTaskDelete(taskHandler);
  //i2s_driver_uninstall(I2S_PORT);
  vTaskDelete(NULL);
}

// 

//Deprecated

void i2s_adc(void *arg)
{
    // i2sInit();
    // size_t bytes_read;
    // i2s_read(I2S_PORT, (void*) i2s_read_buff, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
    // i2s_read(I2S_PORT, (void*) i2s_read_buff, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
    
    Serial.println(" *** Recording Start *** ");
    while (1) {

        // i2s_read(I2S_PORT, (void*) i2s_read_buff, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
        // i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, I2S_READ_LEN);
        // SerialBT.write((const byte*) flash_write_buff, I2S_READ_LEN);

        // for a more accurate reading you could read multiple samples here

        // read a sample from the adc using GPIO35
        int sample = adc1_get_raw(ADC1_CHANNEL_7);
        // get the calibrated value
        Serial.println(sample);
        //Serial.printf("Sample=%d, mV=%d\n", sample, milliVolts);


        SerialBT.write(sample);
        //ets_printf("Never Used Stack Size: %u\n", uxTaskGetStackHighWaterMark(NULL));
    }

    vTaskDelete(NULL);
}

// void i2sInit(){
//   i2s_config_t i2s_config = {
//     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
//     .sample_rate = I2S_SAMPLE_RATE,
//     .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
//     .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
//     .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
//     .intr_alloc_flags = 0,
//     .dma_buf_count = 8,
//     .dma_buf_len = 64,
//     .use_apll = 1
//   };

//   i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

//   const i2s_pin_config_t pin_config = {
//     .bck_io_num = I2S_SCK,
//     .ws_io_num = I2S_WS,
//     .data_out_num = -1,
//     .data_in_num = I2S_SD
//   };

//   i2s_set_pin(I2S_PORT, &pin_config);
// }

// void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
// {
//     uint32_t j = 0;
//     uint32_t dac_value = 0;
//     for (int i = 0; i < len; i += 2) {
//         dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
//         d_buff[j++] = 0;
//         d_buff[j++] = dac_value * 256 / 2048;
//     }
// }
// Task to write samples to our server
// void writerTask(void *param)
// {
//   sampler = (I2SSampler *)param;
//   const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
//   while (true)
//   {
//     // wait for some samples to save
//     uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
//     if (ulNotificationValue > 0)
//     {
//       // send them off to the server
//        SerialBT.println((int)sampler->sampleBuffer());
//     }
//   }
// }


