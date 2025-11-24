## grblHAL-Pendant-Plugin

* This plugin connects grblHAL to an ESP32 over Serial1.
* The ESP32 provides a BLE link to a wireless pendant.
* The plugin processes pendant commands and returns machine status data.

## grblHA L⇄ Serial ⇄ ESP32  ⇄ BLE ⇄ Pendant

* Repository of the ESP32: https://github.com/agent-r/grblHAL-Pendant-BLE-Sender-ESP32.git 
* Repository of the Pendant: https://github.com/agent-r/grblHAL-Pendant.git 

## how to include into grblHAL:

1) copy pendant.c and pendant.h to a folder /src/grblHAL-Pendant-Plugin

2) insert the following code into your my_machine.h

```
#define N_AXIS 4                      //  grbl/config.h
#define OVERRIDE_BUFSIZE 128          //  grbl.hal/override.h

#define PENDANT_ENABLE      1         //  grbl/plugins_init.h

#define SERIAL_PORT	6    // PORT 6 routes to PIN 0 and 1
// #define UART1_RX    	(0u) // not used, info only. for BLE-ESP32
// #define UART1_TX    	(1u) // not used, info only. for BLE-ESP32
```

3) insert the following code at the beginnig of file: grbl/plugins_init.h

```
    #if PENDANT_ENABLE
       extern void pendant_init (void);
       pendant_init();
    #endif
```

