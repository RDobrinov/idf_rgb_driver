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

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
//#include "esp_log.h"

#include "idf_rgb_driver.h"

#define RGBDRV_RESOLUTION_HZ 10000000

/**
 * @brief RBG type with precentage values
 */
typedef struct {
    float r;    /*!< Red channel value 0.0-1.0 */
    float g;    /*!< Green channel value 0.0-1.0 */
    float b;    /*!< Blue channel value 0.0-1.0 */
} rgbdrv_nrgb_t;

/**
 * @brief Gradient mixer type
 */
typedef struct {
    rgbdrv_rgb_t to_rgb_color;  /*!< New RGB color */
    rgbdrv_rgb_t mixed_rgb;     /*!< Mixed RBG color */
    rgbdrv_nrgb_t from_color;   /*!< Gradient start color */
    rgbdrv_nrgb_t to_color;     /*!< Gradient end color */
    float mix_value;        /*!< Current mix value 0.0-1.0 */
    float mix_step;         /*!< Gradient mix step */
} rgbdrv_gradient_t;

/**
 * @brief Task state type
 */
typedef struct {
    TaskHandle_t tsk_handle;        /*!< Led task handle (NOT USED) */
    SemaphoreHandle_t xSemaphore;   /*!< Task semaphore handle. Block access to led program memory */
    bool gradient_run;              /*!< Gradient fade in progress yes/no. Control main task loop */
    TickType_t cycle_start;         /*!< Single led step start time in task ticks */
    TickType_t cycle_interval;      /*!< Single led step interval length in task ticks */
    TickType_t gradient_start;      /*!< Gradient start in task ticks */
    TickType_t gradient_time;       /*!< Gradient interval length in task ticks */
    TickType_t current_tick;        /*!< Current task tick. To avoid multiple gradient calculations */
    TickType_t last_tick;           /*!< Last task tick */
    rgbdrv_gradient_t *mixer;           /*!< Pointer to gradient mix params */
    uint8_t pgmIndex;               /*!< Current program step */
    uint8_t maxIndex;               /*!< Number of program steps */
    rgbdrv_led_state_t *led_pgm;       /*!< Pointer to an array holding led states */
    struct {
        rmt_channel_handle_t rmt_channel;       /*!< RMT channel handler */
        rmt_encoder_handle_t rmt_encoder;       /*!< RMT encoder handler */
        rmt_transmit_config_t rmt_tx_config;    /*!< RMT tx channel config */
    } rmt_driver;
} rgbdrv_task_ctrl_t;

/**
 * @brief Internal RMT encoder type
 */
typedef struct {
    rmt_encoder_t base;             /*!< RMT base encoder*/
    rmt_encoder_t *bytes_encoder;   /*!< RTM bytes encoder*/
    rmt_encoder_t *copy_encoder;    /*!< RTM copy encoder*/
    rmt_encode_state_t internal_state; /*!< Internal encoder state*/
    rmt_symbol_word_t reset_code;   /*!< Reset code/reason*/
} rgbdrv_encoder_t;

static rgbdrv_task_ctrl_t *tc;      /*!< Running task states*/

/**
 * @brief Inverse Srgb Companding for single RGB component
 *
 * @param[in] v R, G or B value
 * @return
 *      - New R, G or B value
 */
static float rgbdrv_ll_value(float v);

/**
 * @brief Srgb Companding for single RGB component
 *
 * @param[in] v R, G or B value
 * @return
 *      - New R, G or B value
 */
static float srgb_companding(float v);

/**
 * @brief Mix two colors loaded int internal driver mixer
 *
 * @return
 *      - Mixed color V ∈ {R,G,B}
 */
static rgbdrv_rgb_t rgbdrv_mix_colors(void);

/**
 * @brief Send new RGB value to LED
 *
 * @param[in] pixel Pointer to {R,G,B} value. R,G and B ∈ [0,255]
 */
static void rgbdrv_set_led_color(rgbdrv_rgb_t *pixel);

/**
 * @brief Reset internal driver counters
 */
static void rgbdrv_reset_cycles(void);

/**
 * @brief Create RMT encoder for encoding LED strip pixels into RMT symbols
 *
 * @param[in] resolution Clock resolution
 * @param[out] _encoder New encoder handler
 * @return
 *      - ESP_OK: Create task successfully
 *      - ESP_ERR_INVALID_ARG: Create RMT TX channel or encoder failed because of invalid argument
 *      - ESP_ERR_NO_MEM: Out of memory
 */
static esp_err_t rgbdrv_new_encoder(const uint32_t resolution, rmt_encoder_handle_t *_encoder);

/**
 * @brief Delete handler
 *
 * @param[in] encoder Base encoder handler
 * @return
 *      - ESP_OK
 * @note No error handling
 */
static esp_err_t rgbdrv_delete_encoder(rmt_encoder_t *encoder);

/**
 * @brief Reset handler
 *
 * @param[in] encoder Base cncoder handler
 * @return
 *      - ESP_OK
 * @note No error handling
 */
static esp_err_t rgbdrv_reset_encoder(rmt_encoder_t *encoder);

/**
 * @brief Encoder handler. Encode symbols
 *
 * @param[in] encoder Encoder handler
 * @param[in] channel RMT Clannel
 * @param[in] rtm_data Data for encoding
 * @param[in] rtm_data_size Data size
 * @param[out] encoder_state New encoder state
 * @return
 *      - Numeber of encoded symbols
 */
static size_t rgbdrv_rtm_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *rtm_data, size_t rtm_data_size, rmt_encode_state_t *encoder_state);

/**
 * @brief Program execute task function
 * 
 * @param[in] pvParameters A NULL value that is passed as the paramater to the created task
 * 
 * @return
*/
static void vDriverTask(void *pvParameters);

esp_err_t rgbdrv_init(gpio_num_t gpio_num) {
    if(tc) return ESP_OK;
    if(!(gpio_num < GPIO_NUM_MAX)) {
        gpio_num = GPIO_NUM_MAX;
        #ifdef CONFIG_IDF_TARGET_ESP32C6
        gpio_num = GPIO_NUM_8;
        #endif
        #ifdef CONFIG_IDF_TARGET_ESP32S3
        gpio_num = GPIO_NUM_48;
        #endif
    }
    if( GPIO_NUM_MAX == gpio_num ) return ESP_ERR_INVALID_ARG;

    esp_err_t init_error = ESP_OK;
    tc = (rgbdrv_task_ctrl_t *)calloc(1, sizeof(rgbdrv_task_ctrl_t));
    if(!tc) return ESP_ERR_NO_MEM;
    /* Init RMT */
    tc->rmt_driver.rmt_channel = NULL;
    tc->rmt_driver.rmt_encoder = NULL;
    tc->rmt_driver.rmt_tx_config = (rmt_transmit_config_t) {.loop_count = 0};

    tc->xSemaphore = xSemaphoreCreateBinary();
    if(!tc->xSemaphore) {
        free(tc);
        return ESP_ERR_NO_MEM;
    } else xSemaphoreGive(tc->xSemaphore);
    // As pointer to rmt_tx_channel_config_t
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio_num,
        .mem_block_symbols = 64,
        .resolution_hz = RGBDRV_RESOLUTION_HZ,
        .trans_queue_depth = 4
    };
    init_error = rmt_new_tx_channel(&tx_chan_config, &tc->rmt_driver.rmt_channel);
    if(ESP_OK != init_error) {
        vSemaphoreDelete(tc->xSemaphore);
        free(tc);
        return init_error;
    }
    rgbdrv_new_encoder(RGBDRV_RESOLUTION_HZ, &tc->rmt_driver.rmt_encoder);
    rmt_enable(tc->rmt_driver.rmt_channel);

    xTaskCreate(vDriverTask, "rgbdrvtsk", 2048, NULL, 12, &tc->tsk_handle);

    return ESP_OK;
}

esp_err_t rgbdrv_set_pgm(rgbdrv_led_state_t *led_state, size_t led_pgm_size) {
    if(ESP_OK != rgbdrv_init(GPIO_NUM_MAX)) return ESP_ERR_INVALID_STATE;
    if(xSemaphoreTake(tc->xSemaphore, (TickType_t) 2) == pdTRUE) {
        free(tc->led_pgm);
        tc->led_pgm = NULL;
        free(tc->mixer);
        tc->mixer = NULL;
        rgbdrv_reset_cycles();
        tc->pgmIndex = 0;
        tc->maxIndex = led_pgm_size;
        if(led_state) {
            size_t mem_size = led_pgm_size * sizeof(rgbdrv_led_state_t);
            tc->led_pgm = (rgbdrv_led_state_t *)malloc(mem_size);
            if(!tc->led_pgm) return ESP_ERR_NO_MEM;
            bool gradient_pgm = false;
            while(!gradient_pgm && (tc->pgmIndex < tc->maxIndex)) {
                gradient_pgm = (led_state[tc->pgmIndex].gradientTime > 0);
                tc->pgmIndex++;
            }
            if(gradient_pgm) {
                tc->mixer = (rgbdrv_gradient_t *)calloc(1, sizeof(rgbdrv_gradient_t));
                if(!tc->mixer) {
                    free(tc->led_pgm);
                    tc->led_pgm = NULL;
                    return ESP_ERR_NO_MEM;
                }
            }
            memcpy(tc->led_pgm, led_state, mem_size);
            tc->pgmIndex = 0;
        }
        xSemaphoreGive(tc->xSemaphore);
    } else return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

static float rgbdrv_ll_value(float v) {
    #if (CONFIG_RGBLED_TASK_GRADIENT == 1)
    return (v > 0.04045f) ? (pow((v + 0.055f) / 1.055f ,2.4f)) : v / 12.92f;
    #else
    return v;
    #endif
}

static float srgb_companding(float v) {
    #if (CONFIG_RGBLED_TASK_GRADIENT == 1)
    return (v > 0.0031308f) ? pow(v, 0.416666666f) * 1.055f - 0.055f : v * 12.92f;
    #else
    return v;
    #endif
}

/*
* Mix from an to colors in task control block
*/
static rgbdrv_rgb_t rgbdrv_mix_colors(void) {
    rgbdrv_rgb_t mixed_color;
    mixed_color.r = (uint8_t)(255.0f * srgb_companding(((tc->mixer->from_color.r) * (1 - tc->mixer->mix_value)) + (tc->mixer->to_color.r * tc->mixer->mix_value))); 
    mixed_color.g = (uint8_t)(255.0f * srgb_companding(((tc->mixer->from_color.g) * (1 - tc->mixer->mix_value)) + (tc->mixer->to_color.g * tc->mixer->mix_value)));
    mixed_color.b = (uint8_t)(255.0f * srgb_companding(((tc->mixer->from_color.b) * (1 - tc->mixer->mix_value)) + (tc->mixer->to_color.b * tc->mixer->mix_value)));
    return mixed_color;
}

static void rgbdrv_set_led_color(rgbdrv_rgb_t *pixel) {
    rmt_transmit(tc->rmt_driver.rmt_channel, tc->rmt_driver.rmt_encoder, pixel, sizeof(rgbdrv_rgb_t), &tc->rmt_driver.rmt_tx_config);
    rmt_tx_wait_all_done(tc->rmt_driver.rmt_channel, portMAX_DELAY);
}

static void rgbdrv_reset_cycles(void) {
    tc->cycle_start = 0;
    tc->cycle_interval = 0;
    tc->cycle_start = 0;
    tc->cycle_interval = 0;
    tc->gradient_run = false;
    tc->gradient_time = 0;
    tc->gradient_start = 0;
    tc->current_tick = 0;
    tc->last_tick = 0;
}

/* RMT Encoder functions */

static esp_err_t rgbdrv_new_encoder(const uint32_t resolution, rmt_encoder_handle_t *_encoder) {
    esp_err_t proc_result = ESP_OK;
    rgbdrv_encoder_t *rgbdrv_encoder = NULL;
    if( !_encoder ) return ESP_ERR_INVALID_ARG;
    rgbdrv_encoder = calloc(1, sizeof(rgbdrv_encoder_t));
    if( !rgbdrv_encoder ) return ESP_ERR_NO_MEM;
    rgbdrv_encoder->base.encode = rgbdrv_rtm_encode;
    rgbdrv_encoder->base.del = rgbdrv_delete_encoder;
    rgbdrv_encoder->base.reset = rgbdrv_reset_encoder;

    /*
    * WS2812B timing
    * 
    * High level +...............+                              +..............................+
    *            .               .                              .                              .
    *            .<0.4us(±150ns)>.        0.85us(±150ns)        .<--------0.8us(±150ns)------->.<0.45us(±150ns)>.
    * Low level ~+---------------+------------------------------+------------------------------+----------------+~
    *            |<-------------------Code 0------------------->|<--------------------Code 1------------------->|
    */
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = { .level0 = 1, .duration0 = 0.4 * resolution / 1000000, .level1 = 0, .duration1 = 0.8 * resolution / 1000000 }, // T0H=0.4us T0L=0.85us
        .bit1 = { .level0 = 1, .duration0 = 0.8 * resolution / 1000000, .level1 = 0, .duration1 = 0.4 * resolution / 1000000 }, // T1H=0.8us T1L=0.45us
        .flags.msb_first = 1 
    };
    proc_result = rmt_new_bytes_encoder(&bytes_encoder_config, &rgbdrv_encoder->bytes_encoder);
    if( ESP_OK == proc_result ) {
        rmt_copy_encoder_config_t copy_encoder_config = {};
        proc_result = rmt_new_copy_encoder(&copy_encoder_config, &rgbdrv_encoder->copy_encoder);
        if( ESP_OK == proc_result ) {
            rgbdrv_encoder->reset_code = (rmt_symbol_word_t) {.level0 = 0, .duration0 = (50 * ( resolution / 1000000 )) , .level1 = 0, .duration1 = 0 };
        }
        else {
            if (rgbdrv_encoder->copy_encoder) {
                rmt_del_encoder(rgbdrv_encoder->copy_encoder);
            }
            if (rgbdrv_encoder->bytes_encoder) {
                rmt_del_encoder(rgbdrv_encoder->bytes_encoder);
            }
            free(rgbdrv_encoder);
            return proc_result;   
        }
    } 
    *_encoder = &rgbdrv_encoder->base;
    return ESP_OK;
}

static esp_err_t rgbdrv_delete_encoder(rmt_encoder_t *encoder) {
    rgbdrv_encoder_t *rgbdrv_encoder = __containerof(encoder, rgbdrv_encoder_t, base);
    rmt_del_encoder(rgbdrv_encoder->bytes_encoder);
    rmt_del_encoder(rgbdrv_encoder->copy_encoder);
    free(rgbdrv_encoder);
    return ESP_OK;
}

static esp_err_t rgbdrv_reset_encoder(rmt_encoder_t *encoder) {
    rgbdrv_encoder_t *rgbdrv_encoder = __containerof(encoder, rgbdrv_encoder_t, base);
    rmt_encoder_reset(rgbdrv_encoder->bytes_encoder);
    rmt_encoder_reset(rgbdrv_encoder->copy_encoder);
    rgbdrv_encoder->internal_state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static size_t rgbdrv_rtm_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *rtm_data, size_t rtm_data_size, rmt_encode_state_t *encoder_state) {
    rgbdrv_encoder_t *rgbdrv_encoder = __containerof(encoder, rgbdrv_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = rgbdrv_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = rgbdrv_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    if( RMT_ENCODING_RESET == rgbdrv_encoder->internal_state ) {
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, rtm_data, rtm_data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            rgbdrv_encoder->internal_state = RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
        }
    }
    if( !(state & RMT_ENCODING_MEM_FULL ) ) {
        if( RMT_ENCODING_COMPLETE == rgbdrv_encoder->internal_state) {
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &rgbdrv_encoder->reset_code, sizeof(rgbdrv_encoder->reset_code), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                rgbdrv_encoder->internal_state = RMT_ENCODING_RESET;
                state |= RMT_ENCODING_COMPLETE;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state |= RMT_ENCODING_MEM_FULL;
            }
        }
    }
    *encoder_state = state;
    return encoded_symbols;
}

static void vDriverTask(void *pvParameters) {
    rgbdrv_reset_cycles();
    while(true)
    {
        if(xSemaphoreTake(tc->xSemaphore, (TickType_t) 1) == pdTRUE) {
            if(tc->led_pgm) {            
                tc->current_tick = xTaskGetTickCount();
                if(tc->gradient_run) {
                    if((tc->current_tick-tc->gradient_start) < tc->gradient_time) {
                        if(tc->last_tick != tc->current_tick) {
                            tc->last_tick = tc->current_tick;
                            rgbdrv_rgb_t new_rgb = rgbdrv_mix_colors();
                            if(new_rgb.r != tc->mixer->mixed_rgb.r || new_rgb.g != tc->mixer->mixed_rgb.g || new_rgb.b != tc->mixer->mixed_rgb.b) {
                                rgbdrv_set_led_color(&new_rgb);
                                tc->mixer->mixed_rgb = new_rgb;
                            }
                            tc->mixer->mix_value += tc->mixer->mix_step;
                        }    
                    } else {
                        tc->gradient_run = false;
                        rgbdrv_set_led_color(&tc->mixer->to_rgb_color);
                    }
                }
                if(!tc->gradient_run){
                    if(tc->cycle_interval < (tc->current_tick - tc->cycle_start))
                    {
                        tc->cycle_interval = ( tc->led_pgm[tc->pgmIndex].holdTime + tc->led_pgm[tc->pgmIndex].gradientTime ) / portTICK_PERIOD_MS;
                        tc->cycle_start = tc->current_tick;
                        if(tc->mixer) {
                            tc->mixer->from_color = tc->mixer->to_color;
                            tc->mixer->to_rgb_color = tc->led_pgm[tc->pgmIndex].color;
                            tc->mixer->to_color.r = rgbdrv_ll_value((float)(tc->led_pgm[tc->pgmIndex].color.r) / 255.0f);
                            tc->mixer->to_color.g = rgbdrv_ll_value((float)(tc->led_pgm[tc->pgmIndex].color.g) / 255.0f);
                            tc->mixer->to_color.b = rgbdrv_ll_value((float)(tc->led_pgm[tc->pgmIndex].color.b) / 255.0f);
                            if(tc->led_pgm[tc->pgmIndex].gradientTime > 0) {
                                tc->mixer->mix_step = 1.0f / ((float)(tc->led_pgm[tc->pgmIndex].gradientTime) / (float)portTICK_PERIOD_MS);
                                tc->mixer->mix_value = 0.0f;
                                tc->gradient_run = true;
                                tc->gradient_time =  tc->led_pgm[tc->pgmIndex].gradientTime / portTICK_PERIOD_MS;
                                tc->gradient_start = tc->current_tick;
                            } else rgbdrv_set_led_color(&tc->led_pgm[tc->pgmIndex].color);
                        } else {
                            rgbdrv_set_led_color(&tc->led_pgm[tc->pgmIndex].color);
                        }
                        tc->pgmIndex++;
                        if(tc->pgmIndex == tc->maxIndex) tc->pgmIndex = 0;
                    }
                }
            }
            xSemaphoreGive(tc->xSemaphore);
        }
        vTaskDelay(1);
    }
}
