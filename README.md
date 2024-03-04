# IDF Single RGB Led control driver

ESP IDF component for single RGB LED control. It has been tested with the ESP32-S3, and ESP32-C6 boards.

![](https://img.shields.io/badge/dynamic/yaml?url=https://raw.githubusercontent.com/RDobrinov/idf_rgb_driver/main/idf_component.yml&query=$.version&style=plastic&color=%230f900f&label)
![](https://img.shields.io/badge/dynamic/yaml?url=https://raw.githubusercontent.com/RDobrinov/idf_rgb_driver/main/idf_component.yml&query=$.dependencies.idf&style=plastic&logo=espressif&label=IDF%20Ver.)
![](https://img.shields.io/badge/-ESP32--S3-rgb(37,194,160)?style=plastic&logo=espressif)
![](https://img.shields.io/badge/-ESP32--C6-rgb(37,194,160)?style=plastic&logo=espressif)
---

## Features

* Thread safe standalone control task
* Gradate or switch to new color

## Installation

1. Create *idf_component.yml*
```shell
idf.py create-manifest
```
2. Edit ***idf_component.yml*** to add dependency
```yaml
dependencies:
  ...
  idf_led_driver:
    version: "main"
    git: git@github.com:RDobrinov/idf_rgb_driver.git
  ...
```
3. Reconfigure project

or 

4. Download and unzip component in project ***components*** folder

### Example
```c
#include <stdio.h>
#include "idf_rgb_driver.h"

void app_main(void)
{
    /* Nice heartbeat blink with fade efect */
    rgbdrv_set_pgm((rgbdrv_led_state_t[]){{{0, 0, 102}, 0, 150}, {{0, 0, 26}, 0, 150}, {{0, 0, 102}, 0, 150}, {{0, 0, 0}, 750, 1500}}, 4);
}
```

### Program element explained

The led program is a sequence of program elements. Each element represents
- Final RGB color state
- Gradate time from previous color
- New color hold time

For example. This program has 4 final states
```c
    /* Nice heartbeat blink with fade efect */
    rgbdrv_set_pgm((rgbdrv_led_state_t[]){{{0, 0, 102}, 0, 150}, {{0, 0, 26}, 0, 150}, {{0, 0, 102}, 0, 150}, {{0, 0, 0}, 750, 1500}}, 4);
```

Last element in example sequence represents new black color gradated frpm previous for 750ms and kept it for 1.5 sec. This create
a nice fade single color efect.
### A note
