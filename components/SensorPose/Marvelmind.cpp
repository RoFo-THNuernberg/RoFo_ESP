#include "Marvelmind.h"

namespace MARVELMIND
{

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
        Marvelmind_Msg_Header header;
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

    Marvelmind::Marvelmind() 
    {   
        ESP_ERROR_CHECK(uart_driver_install(_uart_port, UART_RX_BUFFER, 0, 0, NULL, 0));

        ESP_ERROR_CHECK(uart_param_config(_uart_port, &_uart_conf));

        ESP_ERROR_CHECK(uart_set_pin(_uart_port, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }

    SensorPose& Marvelmind::init()
    {
        if(_sensor == nullptr)
            _sensor = new Marvelmind;

        return *_sensor;
    }

    ros_msgs::Pose2D Marvelmind::get_Pose() {

        uint8_t rx_buffer[UART_RX_BUFFER];

        int len = uart_read_bytes(_uart_port, rx_buffer, UART_RX_BUFFER, 0);

        for(int i = 0; i < len;)
        {

            Marvelmind_Msg_Header *new_data_header = (Marvelmind_Msg_Header*)(rx_buffer + i);

            if(len > 0 && new_data_header->destination_addr == 0xFF && new_data_header->packet_type == 0x47)
            {

                switch(new_data_header->packet_identifier)
                {
                    case 0x0011:
                        Marvelmind_Rx_Data *new_data = (Marvelmind_Rx_Data*)(rx_buffer + i);

                        _current_pose.x = static_cast<float>(new_data->x_coordinate_mm) / 10;
                        _current_pose.y = static_cast<float>(new_data->y_coordinate_mm) / 10;
                        _current_pose.theta = static_cast<float>(new_data->hedgehog_orientation_raw & 0xFFF) / 10;

                        break;
                }
            }

            i += new_data_header->packet_size + sizeof(Marvelmind_Msg_Header);    
        }

        return _current_pose;
    }
}