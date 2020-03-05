# Prj_SLM_nrf52_client

Sample nRF52 client for NCS Serial_LTE_Modem project:  
https://github.com/NordicPlayground/fw-nrfconnect-nrf/tree/master/samples/nrf9160/serial_lte_modem  
NOTE must set the configuration options ``CONFIG_SLM_GPIO_WAKEUP`` and ``CONFIG_SLM_CONNECT_UART_2``.  

Please follow above link for pin inter-connection.

UART instance in use:

    * nRF52840 and nRF52832 (UART0)
    * nRF9160 (UART2)

UART configuration:

    * Hardware flow control: enabled
    * Baud rate: 115200
    * Parity bit: no

slm_client: This sample client is based on "nRF5_SDK_16.0.0\examples\peripheral\bsp".  
  Some hardcoded AT commands could be issued by pushing buttons.

slm_ble_app_uart: This sample client is based on "nRF5_SDK_16.0.0\examplesble_peripheral\ble_app_uart".
  Connect to the NUS client from nRF Toolbox/UART on Android then issue AT commands from the log window.

Note that the GPIO output level on nRF91 side should be 3V (check SW11).

# About this project

This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty.

However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub.

The application is built to be used with the official nRF5 SDK, that can be downloaded from developer.nordicsemi.com

Please post any questions about this project on devzone
