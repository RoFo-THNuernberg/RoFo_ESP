#include "esp_err.h"

#include "SensorPose.h"
#include "Marvelmind.h"
#include "data_types.h"

extern "C" void app_main(void)
{   
    SensorPose *pose_sensor = new Marvelmind();
    ESP_ERROR_CHECK(pose_sensor->init());

    while(1) 
    {
        data_types::Pose x = pose_sensor->get_Pose();
        printf("X: %f, Y: %f, Theta: %f\n", x.x, x.y,  x.theta);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
