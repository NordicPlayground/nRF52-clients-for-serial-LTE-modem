# Prj_SLM_nrf52_client

Sample nRF52 client for NCS Serial_LTE_Modem project:  
https://github.com/NordicPlayground/fw-nrfconnect-nrf/tree/master/samples/nrf9160/serial_lte_modem  
NOTE must set the configuration options ``CONFIG_SLM_GPIO_WAKEUP`` and ``CONFIG_SLM_CONNECT_UART_2``.  

The pin interconnection between nRF91 and nRF52 is presented in the following table:

.. list-table::
   :align: center
   :header-rows: 1

   * - nRF52 DK
     - nRF91 DK
   * - UART TX P0.6
     - UART RX P0.11
   * - UART RX P0.8
     - UART TX P0.10
   * - UART CTS P0.7
     - UART RTS P0.12
   * - UART RTS P0.5
     - UART CTS P0.13
   * - GPIO OUT P0.27
     - GPIO IN P0.31

UART instance in use:

    * nRF52840 and nRF52832 (UART0)
    * nRF9160 (UART2)

UART configuration:

    * Hardware flow control: enabled
    * Baud rate: 115200
    * Parity bit: no
    * Operation mode: IRQ

slm_client: This sample client is based on "nRF5_SDK_16.0.0\examples\peripheral\bsp".  
  Some hardcoded AT commands could be issued by pushing buttons.

slm_ble_app_uart: This sample client is based on "nRF5_SDK_16.0.0\examplesble_peripheral\ble_app_uart".
  Connect to the NUS client from nRF Toolbox/UART on Android then issue AT commands from the log window.

Note that the GPIO output level on nRF91 side should be 3 V.

Testing
=======

When testing the sample with an nRF52 client, the DKs go through the following start-up sequence:

    1. nRF91 starts up and enters sleep state.
    #. nRF52 starts up and starts a periodical timer to toggle the GPIO interface.
    #. nRF52 deasserts the GPIO interface.
    #. nRF91 is woken up and sends a ``Ready\r\n`` message to the nRF52.
    #. On receiving the message, nRF52 can proceed to issue AT commands.

