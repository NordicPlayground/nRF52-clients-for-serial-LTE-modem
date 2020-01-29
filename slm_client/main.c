/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup bsp_example_main main.c
 * @{
 * @ingroup bsp_example
 * @brief BSP Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrf_serial.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define WAKEUP_GPIO_PIN   27

#define NULL_TERMINATION	"\0"
#define CR_TERMINATION		"\r"
#define LF_TERMINATION		"\n"
#define CR_LF_TERMINATION	"\r\n"

#define SYNC_UP_MESSAGE		"Ready"

#define CMD_AT_1    "AT+CGMR"
#define CMD_AT_2    "AT#XSLMVER"
#define CMD_AT_3    "AT#XSOCKET?"
#define CMD_AT_4    "AT#XSLEEP=1"

#define DEV_CTRL_MAX_DATA_LEN   128 

/**< Serial port definitions */
#define SERIAL_FIFO_TX_SIZE     DEV_CTRL_MAX_DATA_LEN * 2
#define SERIAL_FIFO_RX_SIZE     DEV_CTRL_MAX_DATA_LEN * 2
#define SERIAL_BUFF_TX_SIZE     DEV_CTRL_MAX_DATA_LEN    /**< bulk sending */
#define SERIAL_BUFF_RX_SIZE     1                        /**< byte-by-byte receiving */

NRF_SERIAL_DRV_UART_CONFIG_DEF(uart0_drv_config,
                      RX_PIN_NUMBER, TX_PIN_NUMBER,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      NRF_UART_HWFC_ENABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);
NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

static void on_serial_evt(struct nrf_serial_s const * p_serial, nrf_serial_event_t event);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, on_serial_evt, NULL);
NRF_SERIAL_UART_DEF(m_serial_uart, 0);

typedef struct
{
    uint8_t           length;                            /* actuall data length of this buffer block */
    uint8_t           data[DEV_CTRL_MAX_DATA_LEN];       /* max length of one data buffer */
} uart_buf_block_t;

static uart_buf_block_t         m_rx_buf;
static uint8_t                  m_rx_size     = 0;
APP_TIMER_DEF(m_timer);

void uart_rx_init(void)
{
  memset(m_rx_buf.data, 0x00, DEV_CTRL_MAX_DATA_LEN);
  m_rx_buf.length = 0x00;
  m_rx_size = 0;
}

void uart_data_receive(uart_buf_block_t * p_rx_buf)
{
  NRF_LOG_INFO("UART RECEIVE, length:%d.", p_rx_buf->length);
  NRF_LOG_HEXDUMP_INFO(p_rx_buf->data, p_rx_buf->length);

  if (strstr((char *)(p_rx_buf->data), SYNC_UP_MESSAGE))
  {
      app_timer_stop(m_timer);
      NRF_LOG_INFO("Sync up");
      nrf_gpio_pin_set(WAKEUP_GPIO_PIN);
  }

  // prepare for next receive
  uart_rx_init();
}

static void serial_receive_data(nrf_serial_t const * p_serial)
{
  // Invalid protocol packet size
  static uint8_t size_to_receive = DEV_CTRL_MAX_DATA_LEN+1;

  nrf_queue_t const * p_rxq = p_serial->p_ctx->p_config->p_queues->p_rxq;
  m_rx_size += nrf_queue_out(p_rxq, &m_rx_buf.data[m_rx_size], SERIAL_BUFF_RX_SIZE);

  if (m_rx_size > 2)
  {
    /*AT responses are always terminated with <CR><LF> */
    if ((m_rx_buf.data[m_rx_size-2] == '\r') &&
        (m_rx_buf.data[m_rx_size-1] == '\n'))
        {
          m_rx_buf.length = m_rx_size;
          uart_data_receive(&m_rx_buf);
        }
  }
  if (m_rx_size >= size_to_receive)
  {
    m_rx_buf.length = size_to_receive;
    uart_data_receive(&m_rx_buf);
  }
}

static void on_serial_evt(nrf_serial_t const * p_serial, nrf_serial_event_t event)
{
  switch (event)
  {
    case NRF_SERIAL_EVENT_TX_DONE:
      NRF_LOG_DEBUG("NRF_SERIAL_EVENT_TX_DONE");
      break;

    case NRF_SERIAL_EVENT_RX_DATA:
      NRF_LOG_DEBUG("NRF_SERIAL_EVENT_RX_DATA");
      serial_receive_data(p_serial);
      break;

    case NRF_SERIAL_EVENT_DRV_ERR:
      NRF_LOG_ERROR("NRF_SERIAL_EVENT_DRV_ERR(0x%08x)", p_serial->p_ctx->error);
      // TO-DO reset UART RX by GPIO
      break;

    case NRF_SERIAL_EVENT_FIFO_ERR:
      NRF_LOG_ERROR("NRF_SERIAL_EVENT_FIFO_ERR");
      // TO-DO reset UART RX by GPIO
      break;

    default:
      break;
  }
}

uint32_t serial_connect_init(void)
{
  uart_rx_init();

  return nrf_serial_init(&m_serial_uart, &uart0_drv_config, &serial_config);
}

/**@brief Function for un-initialization */
uint32_t serial_connect_uninit(void)
{
  return nrf_serial_uninit(&m_serial_uart);
}

uint32_t inter_connect_send(const uint8_t *data_buff, uint8_t data_len)
{
  uint32_t ret_code;

  NRF_LOG_INFO("UART SEND, length:%d", data_len);
  NRF_LOG_HEXDUMP_INFO(data_buff, data_len);
  
  ret_code = nrf_serial_write(&m_serial_uart, data_buff, data_len, NULL, 0);
  if (ret_code != NRF_SUCCESS)
  {
    return ret_code;
  }

  return nrf_serial_flush(&m_serial_uart, 0);
}

void bsp_evt_handler(bsp_event_t evt)
{
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            inter_connect_send((const uint8_t *)CMD_AT_1, sizeof(CMD_AT_1)-1);
            inter_connect_send((const uint8_t *)CR_LF_TERMINATION, 2);
            break;

        case BSP_EVENT_KEY_1:
            inter_connect_send((const uint8_t *)CMD_AT_2, sizeof(CMD_AT_2)-1);
            inter_connect_send((const uint8_t *)CR_LF_TERMINATION, 2);
            break;

        case BSP_EVENT_KEY_2:
            inter_connect_send((const uint8_t *)CMD_AT_3, sizeof(CMD_AT_3)-1);
            inter_connect_send((const uint8_t *)CR_LF_TERMINATION, 2);
            break;

        case BSP_EVENT_KEY_3:
            inter_connect_send((const uint8_t *)CMD_AT_4, sizeof(CMD_AT_4)-1);
            inter_connect_send((const uint8_t *)CR_LF_TERMINATION, 2);
            break;

        default:
            return; // no implementation needed
    }
}


/**@brief Function for initializing low frequency clock.
 */
void clock_initialization()
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}

static void timer_handle(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    nrf_gpio_pin_toggle(WAKEUP_GPIO_PIN);
}

/**@brief Function for initializing bsp module.
 */
void bsp_configuration()
{
    uint32_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    clock_initialization();

    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = serial_connect_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("SLM client started.");
    bsp_configuration();

    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handle);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
    // wake up nRF91 now
    nrf_gpio_cfg_output(WAKEUP_GPIO_PIN);
    nrf_gpio_pin_clear(WAKEUP_GPIO_PIN);

    while (true)
    {
        NRF_LOG_FLUSH();
        __SEV();
        __WFE();
        __WFE();
        // no implementation needed
    }
}


/** @} */
