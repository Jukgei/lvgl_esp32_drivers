#include "CST816.h"

//TODO: handle multibyte read and refactor to just one read transaction

esp_err_t i2c_cst_read_data(uint8_t addr, uint8_t * data, uint16_t len){
    i2c_ack_type_t last_nack = I2C_MASTER_LAST_NACK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t result = i2c_master_start(cmd);
    
    result = i2c_master_write_byte(cmd, 0x2A, I2C_MASTER_ACK);
    
    result = i2c_master_write_byte(cmd, addr, I2C_MASTER_ACK);
    
    result = i2c_master_start(cmd);
    
    result = i2c_master_write_byte(cmd, 0x2B, I2C_MASTER_ACK);
    
    /* use last_nack, becasue i2c_master_ack will timeout*/
    result = i2c_master_read(cmd, data, len, last_nack);
    
    result = i2c_master_stop(cmd);
    
    result = i2c_master_cmd_begin(I2C_NUM_1, cmd, 4);
		

		// if (!(reg & I2C_NO_REG)) {
		// 	printf("I2C have REG\n");
		// 	/* When reading specific register set the addr pointer first. */
		// 	i2c_master_start(cmd);
		// 	i2c_send_address(cmd, addr, I2C_MASTER_WRITE);
		// 	i2c_send_register(cmd, reg);
		// }
		// /* Read size bytes from the current pointer. */
		// // i2c_master_start(cmd);
		// i2c_send_address(cmd, addr, I2C_MASTER_READ);
		// i2c_master_read(cmd, buffer, size, I2C_MASTER_LAST_NACK);
		// i2c_master_stop(cmd);
		// result = i2c_master_cmd_begin(port, cmd, timeout);


    i2c_cmd_link_delete(cmd);
    return result;
}

void i2c_init()
{
    // 1. Configuration
    i2c_config_t i2c_param = {
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 50000,
        .sda_io_num = 33,
        .scl_io_num = 15,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &i2c_param));


    // 2. Install Driver
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1,I2C_MODE_MASTER,0,0,0));
}

/**
  * @brief  Initialize for CST816T communication via I2C
  * @param  None
  * @retval None
  */
void cst816_init() {

    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<10),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_config);

    i2c_init();

    gpio_set_level(10, 1);

    vTaskDelay(pdMS_TO_TICKS(5));

    gpio_set_level(10, 0);

    vTaskDelay(pdMS_TO_TICKS(5));

    gpio_set_level(10, 1);

    vTaskDelay(pdMS_TO_TICKS(150));

    printf("CST816 INIT CALL\n");
    // unsigned chart dev_addr = 0x01;
    uint8_t data;
    
    // esp_err_t err1 = cst816_i2c_read(0x2B, 0xA7, &data, 1);
    esp_err_t err1 = i2c_cst_read_data( 0xA7, &data, 1);
    printf("Chip id %d, err %d\n", data, err1);

    // esp_err_t err2 = cst816_i2c_read(0x2B, 0xA8, &data, 1);
    esp_err_t err2 = i2c_cst_read_data( 0xA8, &data, 1);
    printf("Proj id %d, err %d\n", data, err2);

    // // esp_err_t err3 = cst816_i2c_read(0x2B, 0xA9, &data, 1);
    esp_err_t err3 = i2c_cst_read_data(0xA9, &data, 1);
    printf("Fw id %d, err %d\n", data, err3);
}


/**
 * Get the current position and state of the touchpad
 * @param data store the read data here
 * @return false: because no more data to be read
 */
bool cst816_touch_read(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    printf("Call touch read\n");
    static int16_t last_x = 0;
    static int16_t last_y = 0;
   	bool touched = true;

    uint8_t finger_num;
    uint8_t x_h, x_l, y_h, y_l;
    uint16_t x;
    uint16_t y;

    esp_err_t err;
    err = i2c_cst_read_data( 0x02, &finger_num, 1);

    err = i2c_cst_read_data( 0x03, &x_h, 1);
    err = i2c_cst_read_data( 0x04, &x_l, 1);

    err = i2c_cst_read_data( 0x05, &y_h, 1);
    err = i2c_cst_read_data( 0x06, &y_l, 1);

    x = (x_h & 0x0f) << 8 | x_l;
    y = (y_h & 0x0f) << 8 | y_l;


	// uint32_t XY = EVE_memRead32(REG_TOUCH_SCREEN_XY);
	// uint16_t Y = XY & 0xffff;
	// uint16_t X = XY >> 16;

	// is it not touched (or invalid because of calibration range)
	// if(X == 0x8000 || Y == 0x8000 || X > LV_HOR_RES_MAX || Y > LV_VER_RES_MAX)
	// {
	// 	touched = false;
	// 	X = last_x;
	// 	Y = last_y;
	// }
	// else
	// {
	// 	last_x = X;
	// 	last_y = Y;
	// }

    data->point.x = x;
    data->point.y = y;
    data->state = (finger_num == 0 ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR);

    printf("point.x: %d, point.y: %d, finger_num: %d\n", x, y, finger_num);

    return false;
}

