#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <sys/time.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <driver/spi_master.h>
#include <cstring>

#define TAG "esp32s3_rgbMatrix"

//A, B, C, D must be on.config.pins.<40

#define PIN_R1 GPIO_NUM_1
#define PIN_G1 GPIO_NUM_2
#define PIN_B1 GPIO_NUM_4
#define PIN_R2 GPIO_NUM_5
#define PIN_G2 GPIO_NUM_6
#define PIN_B2 GPIO_NUM_7
#define EXTRA_SPI_DATAPIN_1 GPIO_NUM_8
#define EXTRA_SPI_DATAPIN_2 GPIO_NUM_9
#define NUM_DATA_PINS 6
#define PIN_A GPIO_NUM_10
#define PIN_B GPIO_NUM_11
#define PIN_C GPIO_NUM_12
#define PIN_D GPIO_NUM_13
#define PIN_E GPIO_NUM_14
#define NUM_ROW_PINS 5
#define PIN_OE GPIO_NUM_15
#define PIN_LAT GPIO_NUM_16
#define PIN_CLK GPIO_NUM_17

#define BUFFER_NUM_PLANES 4
#define PANEL_FRAMERATE 60
#define PANEL_NUM_ELECTRICAL_ROWS 8
#define PANEL_ELECTRICAL_ROW_LENGTH 256 //top half of panel only, as we have two sets of rgb pins

#define PANEL_PX_WIDTH 128
#define PANEL_PX_HEIGHT 32

#define SPI_CLOCK_HZ (20 * 1000000) //20KHz
#define SPI_CS_PIN 17

    static inline __attribute__((always_inline)) 
void directWriteLow(uint8_t pin) 
{     
    if ( pin < 32 ) GPIO.out_w1tc = ((uint32_t)1 << pin);     
    else if ( pin < 40 ) GPIO.out1_w1tc.val = ((uint32_t)1 << (pin - 32)); 
}  

    static inline __attribute__((always_inline)) 
void directWriteHigh(uint8_t pin) 
{     
    if ( pin < 32 ) GPIO.out_w1ts = ((uint32_t)1 << pin);     
    else if ( pin < 40 ) GPIO.out1_w1ts.val = ((uint32_t)1 << (pin - 32)); 
}



struct 
{
    struct {
        struct {
            uint8_t rowPins[NUM_ROW_PINS] = {
                PIN_A,
                PIN_B,
                PIN_C,
                PIN_D,
                PIN_E,
            };
            uint8_t numRowPins = 5;
            uint8_t OE = PIN_OE,
                    LAT = PIN_LAT;
        } pins;
        struct {
            spi_bus_config_t busConfig = {
                .data0_io_num = EXTRA_SPI_DATAPIN_2,
                .data1_io_num = EXTRA_SPI_DATAPIN_1,
                .sclk_io_num = PIN_CLK,
                .data2_io_num = PIN_B2,
                .data3_io_num = PIN_G2,
                .data4_io_num = PIN_R2,
                .data5_io_num = PIN_B1,
                .data6_io_num = PIN_G1,
                .data7_io_num = PIN_R1,
                .flags = SPICOMMON_BUSFLAG_OCTAL, //this sets all data pins to outputs
            };
            spi_host_device_t host = SPI2_HOST;
            spi_device_interface_config_t devConfig = {
                .mode = 0,
                .clock_speed_hz = SPI_CLOCK_HZ,
                .spics_io_num = -1,
                .flags = (SPI_DEVICE_HALFDUPLEX),
                .queue_size = 1,
            };
            spi_device_handle_t devHandle = NULL;
            spi_transaction_t transactionPrototype = {
                .flags = (SPI_TRANS_MODE_OCT),
                .length = PANEL_ELECTRICAL_ROW_LENGTH*8, ///< Total data length, in bits
                .tx_buffer = NULL, ///< Pointer to transmit buffer, or NULL for no MOSI phase
                .rx_buffer = NULL, ///< Pointer to receive buffer, or NULL for no MISO phase. Written by 4 bytes-unit if DMA is used.
            };
            TaskHandle_t spi_tx_task = NULL;
        } spi;
        uint8_t numElecRows = PANEL_NUM_ELECTRICAL_ROWS;
        uint8_t numImgPlanes = BUFFER_NUM_PLANES;
        struct {
            timer_group_t group = TIMER_GROUP_0;
            timer_idx_t id = TIMER_0;
            timer_config_t config = {
                .alarm_en = TIMER_ALARM_EN,
                .counter_en = TIMER_PAUSE,
                .counter_dir = TIMER_COUNT_UP,
                .auto_reload = TIMER_AUTORELOAD_EN,
                .divider = 80000000/(PANEL_FRAMERATE*PANEL_NUM_ELECTRICAL_ROWS*((1<<BUFFER_NUM_PLANES)-1)*1000), //convert 80MHZ base clock to a clock with a tick rate of 1000* the desired framerate
            };
            uint64_t initAlarmVal = 100;
        } timer;
        uint32_t frameRate = PANEL_FRAMERATE;
        uint32_t bufferSize = PANEL_NUM_ELECTRICAL_ROWS*BUFFER_NUM_PLANES*PANEL_ELECTRICAL_ROW_LENGTH;
        uint8_t* buffer[2] = {NULL, NULL};
    } config;
    struct {
        uint8_t currentElecRow = 0,
                nextElecRow = 0,
                currentImgPlane = 0,
                nextImgPlane = 0,
                firstElecRowLastDisplayedPlane = 0,
                dataSendDone = 1,
                activeBuffer = 0,
                numSpiTransactionsInflight = 0;
    } state;
} dev;


void spiTransmitTask(void* params) {
    for(;;)
    {
        if (ulTaskGenericNotifyTake(0, true, portMAX_DELAY)) {
            spi_transaction_t* rcvTrans = NULL;
            for (int i = 0; i < dev.state.numSpiTransactionsInflight; i++) {
                spi_device_get_trans_result(dev.config.spi.devHandle, &rcvTrans, portMAX_DELAY);
                dev.state.numSpiTransactionsInflight--;
            }
            spi_transaction_t trans = dev.config.spi.transactionPrototype;

            uint8_t newElecRow = dev.state.currentElecRow + 1, newImgPlane;
            if (newElecRow == dev.config.numElecRows) {
                newElecRow = 0;
                dev.state.firstElecRowLastDisplayedPlane++;
                if (dev.state.firstElecRowLastDisplayedPlane == dev.config.numImgPlanes)
                    dev.state.firstElecRowLastDisplayedPlane = 0;
                newImgPlane = dev.state.firstElecRowLastDisplayedPlane;
            }
            else {
                newImgPlane = dev.state.currentImgPlane + 1;
                if (newImgPlane == dev.config.numImgPlanes) 
                    newImgPlane = 0;
            }
            dev.state.nextElecRow = newElecRow;
            dev.state.nextImgPlane = newImgPlane;
            //set pointer offset
            trans.tx_buffer = dev.config.buffer[dev.state.activeBuffer]
                + (((dev.state.nextElecRow * BUFFER_NUM_PLANES) + dev.state.nextImgPlane) * (PANEL_ELECTRICAL_ROW_LENGTH * sizeof(uint8_t)));

            dev.state.numSpiTransactionsInflight++;
            esp_err_t err = spi_device_queue_trans(dev.config.spi.devHandle, &trans, 0);
            if (err != ESP_OK) ESP_LOGE(TAG, "spi trans err: %s", esp_err_to_name(err));
            //ESP_LOGI(TAG, "spi tx row %d, plane %d, pointer %p, len %d", dev.state.nextElecRow, dev.state.nextImgPlane, trans.tx_buffer, trans.length);
        }
    }
}


//timer process:
//
//stage 0:
//0. disable timer interrupts
//1. check data send is done  - if not, report it in some way and set the timer period to the minimum, skip the rest
//2. set data sent to not done
//3. set timer period
//4. re-enable timer interrupts
//
//stage 1:
//1. disable output
//2. enable latch to get new data
//3. change row
//4. disable latch
//5. enable output
//
//stage 2:
//7. send new data to shift register - this will have its own interrupt when done to change the state of data sent to done

//boolean return type for timer callback. return true to yield from isr, otherwise dont
uint32_t frameCounter = 0;

static IRAM_ATTR bool timerISR(void* args)
{
    timer_group_t gr = dev.config.timer.group;
    timer_idx_t id = dev.config.timer.id;
    timer_disable_intr(gr, id);

    if (!dev.state.dataSendDone) {
        //throw error
        timer_set_alarm_value(gr, id, 100); //comeback later
        timer_enable_intr(gr, id);
        return false; //no new task was woken just yet
    }

    dev.state.dataSendDone = false;

    //need to work these out now to get timer period
    const uint32_t newTimerPeriodMultiplier = 1<<dev.state.nextImgPlane;
    timer_set_alarm_value(gr, id, newTimerPeriodMultiplier * 1000);

    timer_enable_intr(gr, id);

    //disable output
    directWriteHigh(dev.config.pins.OE);
    //enable latch
    directWriteHigh(dev.config.pins.LAT);
    //change row
    for (uint8_t i = 0; i< dev.config.pins.numRowPins; i++) {
        if ((dev.state.nextElecRow>>i) & 1) 
            directWriteHigh(dev.config.pins.rowPins[i]);
        else directWriteLow(dev.config.pins.rowPins[i]);
    }
    //disable latch
    directWriteLow(dev.config.pins.LAT);
    //enable output
    directWriteLow(dev.config.pins.OE);
    dev.state.currentElecRow = dev.state.nextElecRow;
    dev.state.currentImgPlane = dev.state.nextImgPlane;
    if ((dev.state.currentElecRow == 0) && (dev.state.currentImgPlane == 0)) {
        frameCounter++;
    }
    BaseType_t higherPrioWoken;
    vTaskGenericNotifyGiveFromISR(dev.config.spi.spi_tx_task, 0, &higherPrioWoken);
    return higherPrioWoken;
}



void IRAM_ATTR spi_trans_cb(spi_transaction_t* trans) {

    dev.state.dataSendDone = 1;
    //set trans flag to done
}

uint8_t* writeBuffer = NULL;
uint32_t bufferSize = dev.config.bufferSize;
void swapBuffers(bool copyBuffer) {
    uint8_t activeBuffer = dev.state.activeBuffer;
    writeBuffer = dev.config.buffer[activeBuffer];
    dev.state.activeBuffer = (activeBuffer+1)&1;
    if (copyBuffer) memcpy(writeBuffer, dev.config.buffer[activeBuffer], dev.config.bufferSize);
}
void generateTransfmormationLUTs();
void initRgbPanel() {
    //alloc mem
    ESP_LOGI(TAG, "allocating two buffers of size %d [%f KB]", dev.config.bufferSize, (float)dev.config.bufferSize/1024);
    dev.config.buffer[0] = (uint8_t*)heap_caps_malloc(dev.config.bufferSize*sizeof(uint8_t), MALLOC_CAP_DMA);
    dev.config.buffer[1] = (uint8_t*)heap_caps_malloc(dev.config.bufferSize*sizeof(uint8_t), MALLOC_CAP_DMA);

    generateTransfmormationLUTs();
    if ((dev.config.buffer[0] == NULL) || (dev.config.buffer[1] == NULL)) ESP_LOGE(TAG, "Failed to allocate DMA buffers");

    //init gpios
    gpio_set_direction(PIN_OE, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LAT, GPIO_MODE_OUTPUT);
    for (int i = 0; i < dev.config.pins.numRowPins; i++) {
        gpio_set_direction((gpio_num_t)dev.config.pins.rowPins[i], GPIO_MODE_OUTPUT);
    }
    //init spi
    dev.config.spi.devConfig.post_cb = &spi_trans_cb;
    ESP_ERROR_CHECK(spi_bus_initialize(dev.config.spi.host, &dev.config.spi.busConfig, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(dev.config.spi.host, &dev.config.spi.devConfig, &dev.config.spi.devHandle)); 

    

    xTaskCreate(&spiTransmitTask, "RGB Matrix spi transmit task", 4096, NULL, configMAX_PRIORITIES-1, &dev.config.spi.spi_tx_task);

    timer_group_t gr = dev.config.timer.group;
    timer_idx_t id = dev.config.timer.id;
    ESP_ERROR_CHECK(timer_init(gr, id, &dev.config.timer.config));
    ESP_ERROR_CHECK(timer_set_counter_value(gr, id, 0));
    ESP_ERROR_CHECK(timer_set_alarm_value(gr, id, dev.config.timer.initAlarmVal));
    ESP_ERROR_CHECK(timer_enable_intr(gr, id));
    ESP_ERROR_CHECK(timer_isr_callback_add(gr, id, &timerISR, NULL, 0));
    ESP_ERROR_CHECK(timer_start(gr, id));
    swapBuffers(false);
    //debug routine
    /*
    for (;;) {
        vTaskDelay(1000/portTICK_RATE_MS);
        ESP_LOGI(TAG, "FPS= %d", frameCounter);
        frameCounter = 0;
    }
    */
}


void clearBuffer() {
    memset(writeBuffer, 0, dev.config.bufferSize);
}



//you can probably hard code these, i like to generate them
static uint16_t* transformPosLUT = NULL;
static uint8_t* shiftRtLUT = NULL;

void generateTransfmormationLUTs() {
    transformPosLUT = (uint16_t*)malloc(PANEL_PX_WIDTH*PANEL_PX_HEIGHT*sizeof(uint16_t));
    shiftRtLUT = (uint8_t*)malloc(PANEL_PX_WIDTH*PANEL_PX_HEIGHT*sizeof(uint8_t));
    for (int i = 0; i < PANEL_PX_HEIGHT; i++) {
        for (int j = 0; j < PANEL_PX_WIDTH; j++) {
            const uint32_t offset = (i*PANEL_PX_WIDTH)+j;
            int shiftRt = 0;
            int elecRow = i;
            if (elecRow >= 16) {
                shiftRt = 3;
                elecRow -=16;
            }

            int elecCol = j + 64;
            if (elecCol >= 128) {
                elecCol += 64;
            }
            if (elecRow >= 8) {
                elecCol -= 64;
                elecRow-=8;   
            }
            
            transformPosLUT[offset] = (elecRow*PANEL_ELECTRICAL_ROW_LENGTH*BUFFER_NUM_PLANES)+elecCol;
            shiftRtLUT[offset] = shiftRt;
        }
    }
}

//use the transformation lookup tables
inline void transformPos(int row, int col, int* ptrOffset, int* shiftRtBy) {
    const int offset = (row*PANEL_PX_WIDTH)+col;
    *ptrOffset = transformPosLUT[(row*PANEL_PX_WIDTH)+col];
    *shiftRtBy = shiftRtLUT[offset];
}


inline void writeByteIntoBuf(int ptrOffset, int plane, uint8_t byte, uint8_t bitMask) {
    uint8_t* ptr = writeBuffer
            + ptrOffset
            + (PANEL_ELECTRICAL_ROW_LENGTH*plane);
    *ptr = ((*ptr) & (~bitMask)) | (byte & bitMask);
}
inline void rgb565ToPlaneBytes(uint16_t color, uint8_t* planeBytes) {
    uint8_t red = (color>>8); //msb to u8 pos 0
    uint8_t green = (color>>4); //msb to u8 pos 1
    uint8_t blue = (color<<1); //msb to u8 pos 2
    for (int i = BUFFER_NUM_PLANES-1; i>=0; i--) {
        planeBytes[i] = ((red & 0b10000000)| (green & 0b01000000) | (blue & 0b00100000));
        red = red << 1;
        green = green << 1;
        blue = blue <<1;
    }
}

void drawPixel(int row, int col, uint16_t color) {

    if ((row < 0) | (col < 0) | (row >= PANEL_PX_HEIGHT) | (col >= PANEL_PX_WIDTH)) return; //invalid loc
    int ptrOffset, shiftRt;
    transformPos(row, col, &ptrOffset, &shiftRt);
    //buffers are filled lsb to msb
    //align red green blue to positions
    uint8_t planeBytes[BUFFER_NUM_PLANES];
    rgb565ToPlaneBytes(color, planeBytes);
    for (int i = 0; i < BUFFER_NUM_PLANES; i++) {
        writeByteIntoBuf(ptrOffset, i, planeBytes[i]>>shiftRt, 0b11100000>>shiftRt);
    }
}
