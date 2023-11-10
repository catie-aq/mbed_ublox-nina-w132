# U-blox NINA-W132 WiFi driver

Mbed OS library for u-blox NINA-W132 WiFi module. Compatible with the NetWorkInterface API.

## Configuration

To use this module as the default NetworkInterface of your application, you need to add
the following configuration parameters to your `mbed_app.json` file:

```
{
    "target_overrides": {
        "*": {
            "target.components_add": ["nina_w132"],
            "target.network-default-interface-type": "WIFI",
            "nina_w132.provide-default": true
        }
    }
}
```

All the configuration parameters are visible in the `mbed_lib.json` file. You can override them using an `mbed_app.json` file.

You should at least define your UART pins and Reset pin in your `mbed_app.json` file. For example:

```
{
    "target_overrides": {
        "*": {
            "nsapi.default-wifi-security": "WPA2",
            "nsapi.default-wifi-ssid": "\"TESTTEST\"",
            "nsapi.default-wifi-password": "\"6tron2020\"",
            "platform.stdio-baud-rate": 115200,
            "mbed-trace.enable": false,
            "mbed-trace.max-level": "TRACE_LEVEL_DEBUG",
            "rtos.main-thread-stack-size": 8192
        },
        "ZEST_CORE_STM32L4A6RG": {
            "target.components_add": ["nina_w132"],
            "target.network-default-interface-type": "WIFI",
            "nina_w132.provide-default": true,
            "nina_w132.tx": "UART1_TX",
            "nina_w132.rx": "UART1_RX",
            "nina_w132.rst": "DIO2",
            "nina_w132.debug": false
        }
    }
}
```

