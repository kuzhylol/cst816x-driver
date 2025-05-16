# Linux Kernel Driver for CST816x Touchscreen Controller

This guide helps you build, install, and set up a kernel module and device tree overlay for the CST816S touchscreen controller on a Raspberry Pi with a 1.28-inch round LCD.

<img src="https://github.com/kuzhylol/cst816x-driver/raw/main/pictures/cst816s-ts.png" width="240" height="180">

---

## 📌 Documentation

### Raspberry Pi 4B to Display Pinout

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

Refer to the [Waveshare guide](https://www.waveshare.com/wiki/1.28inch_Touch_LCD) for hardware connection.

### Input Device Capabilities

| Event Code         | Description       |
|--------------------|-------------------|
| BTN_LEFT           | Gesture Left      |
| BTN_RIGHT          | Gesture Right     |
| BTN_FORWARD        | Gesture Up        |
| BTN_BACK           | Gesture Down      |
| BTN_TOUCH          | Single Touch      |
| BTN_TOOL_TRIPLETAP | Long Press (Hold) |

---

## 🧪 Testing Environment

- [GC9A01 FBTFT Overlay](https://github.com/juliannojungle/gc9a01-overlay)
- [LVGL Demo](https://github.com/lvgl/lv_port_linux)

### Enable Frame Buffer in `lv_conf.h`
```c
#define LV_USE_LINUX_FBDEV 1
```

### Initialize Frame Buffer
```c
const char *device = getenv_default("LV_LINUX_FBDEV_DEVICE", "/dev/fbN");
lv_display_t *disp = lv_linux_fbdev_create();
lv_linux_fbdev_set_file(disp, device);
```

### Initialize Input Device
```c
lv_indev_t *input = lv_evdev_create(LV_INDEV_TYPE_POINTER, "/dev/input/eventN");
lv_indev_set_display(input, disp);
```

### Main Loop
```c
ui_init();
while (1) {
    format_linux_date();
    lv_timer_handler();
    usleep(1000);
}
```

---

## 🧱 Kernel Module Setup

### Build as External Module

```sh
cd cst816x-driver/
make
# Or manually specify:
make -C <path_to_kernel_sources> M=$PWD
```

### Install the Module

```sh
make install
# Or manually specify:
make -C <path_to_kernel_sources> M=$PWD install
```

### Clean Build Files

```sh
make clean
# Or manually specify:
make -C /home/pi/linux-6.9.y M=$PWD clean
```

### Load the Module

```sh
sudo insmod hynitron-cst816x.ko
dmesg
```

Example output:
```plaintext
[1593.107] hynitron_cst816x: loading out-of-tree module taints kernel.
[1593.283] input: Hynitron CST816X Touchscreen as /devices/...
```

---

## 🔧 Built-in Kernel Driver

### Kconfig Addition
```plaintext
config TOUCHSCREEN_HYNITRON_CST816X
    tristate "Hynitron CST816X touchscreen support"
    depends on I2C
    help
      Say Y if you have a Hynitron CST816X touchscreen.
```

### Makefile Addition
```make
obj-$(CONFIG_TOUCHSCREEN_HYNITRON_CST816X) += hynitron-cst816x.o
```

---

## 🌱 Device Tree Setup

### Example DTS Snippet

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

### Raspberry Pi DTS Overlay

```sh
cd cst816x-driver/rpi4b/
dtc -@ -I dts -O dtb -o cst816s.dtbo hynitron-cst816s.dts
sudo cp cst816s.dtbo /boot/overlays/
echo "dtoverlay=cst816s" | sudo tee -a /boot/config.txt
sudo reboot
```

---

## 🧪 Verify Input Device

```sh
evtest /dev/input/eventN
```

Expected output:
```plaintext
Input device name: "Hynitron CST816X Touchscreen"
Supported events:
  EV_SYN
  EV_KEY: BTN_LEFT, BTN_RIGHT, BTN_FORWARD, BTN_BACK, BTN_TOUCH, BTN_TOOL_TRIPLETAP
  EV_ABS: ABS_X (0-240), ABS_Y (0-240)
```

