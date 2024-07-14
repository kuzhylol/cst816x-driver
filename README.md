# Linux Kernel driver for CST816S Touchscreen controller based on 1.28 Inch Round LCD display

This guide will help you build, install, and set up your kernel module and CST816S Touch Screen device tree overlay for a Raspberry Pi.

<img src="https://github.com/kuzhylol/cst816x-driver/raw/main/pictures/cst816s-ts.png" width="240" height="180">

## Documentation

**Raspberry Pi x Round display pinout**
| Module Pin | Raspberry Pi (BCM) |
|------------|--------------------|
| VCC        | 3.3V               |
| GND        | GND                |
| MISO       | 9                  |
| MOSI       | 10                 |
| SCLK       | 11                 |
| LCS_CS     | 8                  |
| LCS_DC     | 25                 |
| LCS_RST    | 27                 |
| LCS_BL     | 18                 |
| TP_SDA     | 2                  |
| TP_SCL     | 3                  |
| TP_INT     | 4                  |
| TP_RST     | 17                 |

Follow the [Detailed guide](https://www.waveshare.com/wiki/1.28inch_Touch_LCD) provided by Waveshare to set up hardware connection.

**Input device Capabilities Description**

| Event Code         | Description           |
|--------------------|-----------------------|
| BTN_LEFT           | Gesture Left          |
| BTN_RIGHT          | Gesture Right         |
| BTN_FORWARD        | Gesture Up            |
| BTN_BACK           | Gesture Down          |
| BTN_TOUCH          | Single Touch          |
| BTN_TOOL_TRIPLETAP | Long press (hold)     |

**Testing environment**
1. [GC9A01 FBTFT overlay](https://github.com/juliannojungle/gc9a01-overlay) to draw graphics
2. [LVGL demo](https://github.com/lvgl/lv_port_linux) to test Touschreen functionality.

Follow the steps to enable LVGL demo:

**Enable Frame buffer**

Need to enable Frame Buffer functionality inside LVGL header config (lv_conf.h).

```c
#define LV_USE_LINUX_FBDEV	1
```

**Initialize Frame buffer**

```c
const char *device = getenv_default("LV_LINUX_FBDEV_DEVICE", "/dev/fbN");
lv_display_t *disp = lv_linux_fbdev_create();
lv_linux_fbdev_set_file(disp, device);
```

**Initialize Input Device (Linux Touchscreen Interface)**

```c
lv_indev_t *input = lv_evdev_create(LV_INDEV_TYPE_POINTER, "/dev/input/eventN");
lv_indev_set_display(input, disp);
```

** Initialize and start LWFS **
```c
ui_init();
while(1) {
    format_linux_date();
    lv_timer_handler();
    usleep(1000);
}
```

## Kernel Module Setup Instructions

### Build driver as External Kernel Module

1. **Navigate to the kernel module directory**:

    ```sh
    cd cst816x-driver/
    ```

2. **Build the module using the default kernel headers**:

    ```sh
    make
    ```

    Alternatively, specify the kernel source directory manually:

    ```sh
    make -C <local_path_to_kernel_sources> M=$PWD
    ```

### Install  driver as External Kernel Module

1. **Install the module using the default kernel headers**:

    ```sh
    make install
    ```

    Alternatively, specify the kernel source directory manually:

    ```sh
    make -C <local_path_to_kernel_sources> M=$PWD install
    ```

### Clean the Build Files

1. **Clean the build files using the default kernel headers**:

    ```sh
    make clean
    ```

    Alternatively, specify the kernel source directory manually:

    ```sh
    make -C /home/pi/linux-6.9.y M=$PWD clean
    ```

### Install as External Module

1. **Insert the module into the kernel**:

    ```sh
    sudo insmod hynitron-cst816x.ko
    ```

2. **Check the module status**:

    ```sh
    dmesg
    ```

    Example output:

    ```plaintext
    [ 1593.107576] hynitron_cst816x: loading out-of-tree module taints kernel.
    [ 1593.283284] input: Hynitron CST816X Touchscreen as /devices/platform/soc/fe804000.i2c/i2c-1/1-0015/input/inputN
    ```

### Built-in Kernel driver
1. Append CST816X entry  to Kernel Kconfig for input/touchscreen section: **drivers/input/touchscreen/Kconfig**

```plaintext
config TOUCHSCREEN_HYNITRON_CST816X
	tristate "Hynitron CST816X touchscreen support"
	depends on I2C
	help
	  Say Y here if you have a touchscreen using a Hynitron
	  CST816X touchscreen controller.

	  If unsure, say N.

	  To compile this driver as a module, choose M here: the
	  module will be called hynitron-cst816x.
```

2. Append module entry to Makefile: **drivers/input/touchscreen/Makefile**

```plaintext
obj-$(CONFIG_TOUCHSCREEN_GOODIX_BERLIN_I2C)    += goodix_berlin_i2c.o
obj-$(CONFIG_TOUCHSCREEN_GOODIX_BERLIN_SPI)    += goodix_berlin_spi.o
obj-$(CONFIG_TOUCHSCREEN_HIDEEP)       += hideep.o
+obj-$(CONFIG_TOUCHSCREEN_HYNITRON_CST816X)     += hynitron-cst816x.o
obj-$(CONFIG_TOUCHSCREEN_ILI210X)      += ili210x.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)       += ilitek_ts_i2c.o
```

3. Build Kernel, driver will included as built-in module

## Device Tree setup steps
### Device Tree approach

Include the following content in your device tree file (e.g., `hynitron-cst816s.dts`):

```dts
	#include <dt-bindings/gpio/gpio.h>
	#include <dt-bindings/interrupt-controller/irq.h>

	i2c {
	    #address-cells = <1>;
	    #size-cells = <0>;
	    touchscreen@15 {
	        compatible = "hynitron,cst816s";
	        reg = <0x15>;
	        interrupt-parent = <&gpio0>;
	        interrupts = <4 IRQ_TYPE_EDGE_RISING>;
	        reset-gpios = <&gpio 17 GPIO_ACTIVE_LOW>;
	    };
	};
```

### Raspberry Pi DTS overlay approach

1. **Navigate to the directory containing your device tree source file**:

    ```sh
    cd cst816x-driver/rpi4b/
    ```

2. **Compile the device tree overlay**:

    ```sh
    dtc -@ -I dts -O dtb -o cst816s.dtbo hynitron-cst816s.dts
    ```
3. **Copy the compiled overlay to the boot overlays directory**:
    ```sh
    sudo cp cst816s.dtbo /boot/overlays/
    ```

2. **Add the overlay to the `config.txt` file**:

    ```sh
    sudo sh -c 'echo "dtoverlay=cst816s" >> /boot/config.txt'
    ```

### Reboot the Device

**Reboot your Raspberry Pi to apply the overlay**:
```sh
	sudo reboot
```

Following these steps will build and install your kernel module and device tree overlay on a Raspberry Pi. Make sure to adjust any paths to match your specific environment.

### Verify Input Device Capabilities
**You can find available capabilities showing that device operate properly**
```plaintext
	evtest /dev/input/eventN
	Input driver version is 1.0.1
	Input device ID: bus 0x18 vendor 0x0 product 0x0 version 0x0
	Input device name: "Hynitron CST816X Touchscreen"
	Supported events:
	  Event type 0 (EV_SYN)
	  Event type 1 (EV_KEY)
	    Event code 272 (BTN_LEFT)
	    Event code 273 (BTN_RIGHT)
	    Event code 277 (BTN_FORWARD)
	    Event code 278 (BTN_BACK)
	    Event code 330 (BTN_TOUCH)
	    Event code 334 (BTN_TOOL_TRIPLETAP)
	  Event type 3 (EV_ABS)
	    Event code 0 (ABS_X)
	      Value      0
	      Min        0
	      Max      240
	    Event code 1 (ABS_Y)
	      Value      0
	      Min        0
	      Max      240
```
