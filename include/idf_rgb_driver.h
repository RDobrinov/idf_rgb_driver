#ifndef _IDF_RGB_DRIVER_
#define _IDF_RGB_DRIVER_

#include "esp_system.h"

/**
 * @brief Type of RGB color
 */
typedef struct {
    uint8_t r;  /*!< Red channel value 0-255 */
    uint8_t g;  /*!< Green channel value 0-255 */
    uint8_t b;  /*!< Blue channel value 0-255 */
} rgbdrv_rgb_t;

/**
 * @brief Type of single step in led program
 */
typedef struct {
    rgbdrv_rgb_t color;         /*!< Pixel RGB */
    uint16_t gradientTime;     /*!< Time to fade from previous color to new, in ms */
    uint16_t holdTime;         /*!< Time to hold new color before start next program step, in ms */
} rgbdrv_led_state_t;

/**
 * @brief Create RMT encoder for encoding LED strip pixels into RMT symbols
 *
 * @return
 *      - ESP_OK: Create task successfully
 *      - ESP_ERR_INVALID_ARG: Create RMT TX channel or encoder failed because of invalid argument
 *      - ESP_ERR_NO_MEM: Out of memory
 *      - ESP_ERR_NOT_FOUND: Create RMT TX channel failed because all RMT channels are used up and no more free one
 *      - ESP_ERR_NOT_SUPPORTED: Create RMT TX channel failed because some feature is not supported by hardware, e.g. DMA feature is not supported by hardware
 *      - ESP_FAIL: Init task failed because of other/unknown error
 */
esp_err_t rgbdrv_init(gpio_num_t gpio_num);

/**
 * @brief Init and create led control task
 *
 * @param[in] led_state Pointer to an array of led states in program. Program must end with holdTime = -1
 * @param[in] led_pgm_size Total size of program in bytes
 * @return
 *      - ESP_ERR_INVALID_STATE Task not running
 *      - ESP_ERR_TIMEOUT Semaphore not taken
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM Out of memory when creating program
 *      - ESP_OK Program applied successfully
 */
esp_err_t rgbdrv_set_pgm(rgbdrv_led_state_t *led_state, size_t led_pgm_size);

#endif /* _IDF_RGB_DRIVER_ */