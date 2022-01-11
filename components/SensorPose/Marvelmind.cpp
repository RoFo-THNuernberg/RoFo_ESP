#include "Marvelmind.h"

#define UART_RX_BUFFER 512
#define UART_TX_PIN 27
#define UART_RX_PIN 26
#define UART_BAUDRATE 115200

#define TAG "Marvelmind_Pose"


//Marvelmind UART communication protcol
//see: https://marvelmind.com/pics/marvelmind_interfaces.pdf

struct __attribute__((packed, aligned(1))) Marvelmind_Msg_Header  
{
    uint8_t destination_addr;
    uint8_t packet_type;
    uint16_t packet_identifier;
    uint8_t packet_size;
};

struct __attribute__((packed, aligned(1)))  Marvelmind_Rx_Data 
{
    uint32_t timestamp_ms;
    int32_t x_coordinate_mm;
    int32_t y_coordinate_mm;
    int32_t z_coordinate_mm;
    uint8_t flags;
    uint8_t hedgehog_addr;
    uint16_t hedgehog_orientation_raw;
    uint16_t time_delay;
    uint16_t crc; 
};

Marvelmind* Marvelmind::_marvelmind_sensor = nullptr;

const uart_port_t Marvelmind::_uart_port{UART_NUM_2};
const uart_config_t Marvelmind::_uart_conf = 
{
    .baud_rate = UART_BAUDRATE,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_APB     
};

Marvelmind::Marvelmind() : _measurement_noise_cov{400, 0, 0, 0, 400, 0, 0, 0, 400}
{   
    ESP_ERROR_CHECK(uart_driver_install(_uart_port, UART_RX_BUFFER, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(_uart_port, &_uart_conf));

    ESP_ERROR_CHECK(uart_set_pin(_uart_port, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(_uart_read_data_task, "_uart_read_data_task", 2048, this, 5, NULL);
}

SensorPose& Marvelmind::init()
{
    if(_marvelmind_sensor == nullptr)
        _marvelmind_sensor = new Marvelmind;

    return *_marvelmind_sensor;
}

ros_msgs::Pose2D Marvelmind::getPose() {
    _new_data_ready = false;

    return _current_pose;
}

bool Marvelmind::newData()
{
    return _new_data_ready;
}
/*
void Marvelmind::calculateKalmanGain(std::array<std::array<float, 3>, 3> const& a_priori_cov)
{
    //float inverse_det = a_priori_cov[0][0]*a_priori_cov[1][1]*a_priori_cov[2][2];

    std::array<std::array<float, 3>, 3> inverse;


    _kalman_gain = a_priori_cov;
}
*/
void Marvelmind::_uart_read_data_task(void* pvParameters)
{
    Marvelmind* marvelmind = (Marvelmind*)pvParameters;
    int status = ESP_OK;

    while(1)
    {

        Marvelmind_Msg_Header msg_header;

        int len = uart_read_bytes(_uart_port, &msg_header, sizeof(Marvelmind_Msg_Header), portMAX_DELAY);

        if(len == -1)
        {
            status = ESP_FAIL;
            break;
        }

        uint8_t data_buffer[msg_header.packet_size];

        len = uart_read_bytes(_uart_port, &data_buffer, msg_header.packet_size, portMAX_DELAY);

        if(len == -1)
        {
            status = ESP_FAIL;
            break;
        }

        switch(msg_header.packet_identifier)
        {
            case 0x0011:
            {
                Marvelmind_Rx_Data* msg_data = (Marvelmind_Rx_Data*) data_buffer;

                marvelmind->_current_pose.x = static_cast<float>(msg_data->x_coordinate_mm) / 10;
                marvelmind->_current_pose.y = static_cast<float>(msg_data->y_coordinate_mm) / 10;
                marvelmind->_current_pose.theta = static_cast<float>(msg_data->hedgehog_orientation_raw & 0xFFF) / 10;

                marvelmind->_new_data_ready = true;

                break;
            }
            default:
                
                break;
        }

    }

    ESP_ERROR_CHECK(status);

    vTaskDelete(NULL);
}