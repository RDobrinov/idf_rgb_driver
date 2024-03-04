/*
 * SPDX-FileCopyrightText: 2024 Rossen Dobrinov
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright 2024 Rossen Dobrinov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
 * @verbatim
 *               apply new led color                          apply new led color
 *                     -------                                      -------
 *                        v                                            v            
 * |  rgbdrv_led_state_t  | rgbdrv_led_state_t  |  rgbdrv_led_state_t  | rgbdrv_led_state_t  |
 * |<----gradientTime---->|<-----holdTime------>|<----gradientTime---->|<-----holdTime------>|
 * |                                            |                                            |
 * |<-------program element full time---------->|<-------program element full time---------->|

 * @endverbatim
 * 
 * @note Program element full time is the sum of [gradientTime] and [holdTime].
 * [gradientTime] is transition duration to new LED color, so zero means none 
 * transition at all. [holdTime] is time interval to hold RGB color before apply
 * next program element.
 * 
 * @example
 * (rgbdrv_led_state_t[]){{{0, 61, 102}, 0, 150}, {{0, 15, 26}, 0, 150}, {{0, 61, 102}, 0, 150}, {{0, 0, 0}, 750, 1500}}
 * {{0, 61, 102}, 0, 150} - Apply new color ( nice sky blue 20% Lighter)
 * {{0, 15, 26}, 0, 150}  - After 150mS apply same color with 5% Lighter
 * {{0, 61, 102}, 0, 150} - After 150mS apply same color with 20% Lighter
 * {{0, 0, 0}, 0, 150}    - After 150mS start transition for 750ms from RGB{0, 61, 102} to RGB{0, 0, 0} 
 *                          then hold for 2 second and restart
*/
/**
 * @brief Type of single step in led program
 */
typedef struct {
    rgbdrv_rgb_t color;         /*!< Pixel RGB */
    uint16_t gradientTime;     /*!< Time to fade from previous color to new, in ms */
    uint16_t holdTime;         /*!< Time to hold new color before start next program step, in ms */
} rgbdrv_led_state_t;

/**
 * @brief Init driver and create driver task
 *
 * @param[in] gpio_num GPIO num connected to RGB Led. Pass GPIO_NUM_MAX for autoconfig (ESP32C6 and ESP32S3 DevBoards only)
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
 * @brief Apply new led program sequence. Pass NULL pointer to clear current program
 *
 * @param[in] led_state Pointer to an array of led states in program.
 * @param[in] led_pgm_size Nummber of program elements
 * @return
 *      - ESP_ERR_INVALID_STATE Task not running
 *      - ESP_ERR_TIMEOUT Semaphore not taken
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM Out of memory when creating program
 *      - ESP_OK Program applied successfully
 */
esp_err_t rgbdrv_set_pgm(rgbdrv_led_state_t *led_state, size_t led_pgm_size);

#endif /* _IDF_RGB_DRIVER_ */