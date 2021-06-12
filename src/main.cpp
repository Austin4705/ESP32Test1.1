#include <Arduino.h>
// #include "I2SSampler.h"
#include "BluetoothSerial.h"
#include <driver/i2s.h>
#include "esp_adc_cal.h"
#include <queue>

BluetoothSerial SerialBT;

// calibration values for the adc
#define sampleRatePerSec 1000
#define DEFAULT_VREF 1100
esp_adc_cal_characteristics_t *adc_chars;


std::queue<uint16_t>* bufferQueue;
bool flag = 0;

void initBT();
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void startRecording();
void finishRecording();
void recordData(void *arg);
void transmitData(void *arg);
void deviceCancel(void *arg);
void logData(void *arg);

TaskHandle_t taskHandler1 = NULL;
TaskHandle_t taskHandler2 = NULL;
unsigned long int timer = 0;
unsigned long int counter = 0;
unsigned long int startTime = 0;

void setup() {
  Serial.begin(115200);
  initBT();

  //------------------------Init ADC------------------------

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

  //------------------------Init ADC------------------------

  // std::queue<uint16_t> internal;
  // bufferQueue = &internal;
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
  Serial.println("The device started, now you can pair it with bluetooth-003");
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
  Serial.println("*** Recording Start ***");
  bufferQueue = new std::queue<uint16_t>;
  xTaskCreate(recordData, "recordData", 1024 * 2, NULL, 3, &taskHandler1);
  xTaskCreate(transmitData, "transmitData", 1024 * 2, NULL, 2, &taskHandler2);
  //xTaskCreate(logData, "logData", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void finishRecording(){
  Serial.println("*** Recording End ***");
  vTaskDelete(taskHandler1);
  flag = 0;
  xTaskCreate(deviceCancel, "deviceCancel", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void logData(void *arg){
  vTaskDelay(2000);
  while(1){
      Serial.print((micros() - timer) / 1000.00);
      Serial.print(", ");
      Serial.print(  counter / (float)((millis() - startTime)/1000.00)  );
      Serial.print(", ");
      Serial.print(counter);
      Serial.print(", ");
      Serial.print((float)((millis() - startTime)/1000.00));
      Serial.print(", ");
      Serial.println(bufferQueue->size());

      // Serial.print(counter);
    // Serial.print(", ");
    // Serial.print((micros() - time2)/1000.00);
    // Serial.print(", ");
    // Serial.print(millis() - startTime);
    // Serial.print(", ");
    // Serial.print(counter / (((timer - startTime)/1000.00)/1000.00) );
    // Serial.print(", ");
    // Serial.println(bufferQueue.size());
    vTaskDelay(2000);
  }
  vTaskDelete(NULL);
}


void deviceCancel(void *arg){
  vTaskDelay(2000);
  while(1){
    if(flag == 0){
      Serial.println("***Transmition Done***");
      vTaskDelete(taskHandler2);
      delete bufferQueue;
      break;
    }
    vTaskDelay(1000);
  }
  vTaskDelete(NULL);
}

void recordData(void *arg){
  Serial.println("Test1");
  timer = micros();
  startTime = millis();
  
  counter = 0;
  while(1){
    //if(   ((micros() - timer) / 1000.00)/1000.00 >= 1 / (float)(sampleRatePerSec)){//+10.7
      timer = micros();
      int sample = adc1_get_raw(ADC1_CHANNEL_7);
      // Serial.print("R");
      //Serial.println(sample);
      counter++;
      bufferQueue->push((uint32_t)sample);
    //}
    vTaskDelay(portTICK_PERIOD_MS);

  }
  vTaskDelete(NULL);
}

void transmitData(void *arg){
  Serial.println("Test2");
  //unsigned long int startTime2 = millis();
  unsigned long int time2 = micros();

  while (1) {
    if(bufferQueue->size() > 0){
    flag = 1;
    int sample = (uint32_t)bufferQueue->front();  
    SerialBT.println(sample);
    //Serial.print("TS");
    //Serial.println(sample);
    bufferQueue->pop();
    time2 = micros();
    }
    else{
      flag = 0;
      //Serial.print("TB");
      //Serial.println(bufferQueue.size());
    }
    vTaskDelay(portTICK_PERIOD_MS/2);
  }
  vTaskDelete(NULL);
}

void loop()
{
}









// 

//Deprecated

// void i2s_adc(void *arg)
// {
//     Serial.println(" *** Recording Start *** ");
//     while (1) {
//         // for a more accurate reading you could read multiple samples here

//         int sample = adc1_get_raw(ADC1_CHANNEL_7);
//         Serial.println(sample);
//     }

//     vTaskDelete(NULL);
// }

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


