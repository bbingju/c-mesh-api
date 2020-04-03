/* Copyright(c) 2020. Vinetech. All rights reserved. */
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"

#define LOG_MODULE_NAME "SERIAL"
#define MAX_LOG_LEVEL INFO_LOG_LEVEL
#include "logger.h"

#define WPC_UART_NUM UART_NUM_2 /* CONFIG_WPC_UART_NUM */

/* #define WPC_SERIAL_TXD         CONFIG_WIREPAS_UART_GPIO_TXD */
/* #define WPC_SERIAL_RXD         CONFIG_WIREPAS_UART_GPIO_RXD */
#define WPC_SERIAL_TXD 10  //17
#define WPC_SERIAL_RXD 9   //16
#define WPC_SERIAL_RTS (UART_PIN_NO_CHANGE)
#define WPC_SERIAL_CTS (UART_PIN_NO_CHANGE)

/* #define BUF_SIZE               CONFIG_WIREPAS_UART_BUFSIZE */
/* #define UART_BAUD_RATE         CONFIG_WIREPAS_UART_BAUD_RATE */
#define BUF_SIZE 2048
#define UART_BAUD_RATE 125000

static bool uart_initialized = false;

static int init_uart()
{
    uart_config_t uart_config = {.baud_rate = UART_BAUD_RATE,
                                 .data_bits = UART_DATA_8_BITS,
                                 .parity = UART_PARITY_DISABLE,
                                 .stop_bits = UART_STOP_BITS_1,
                                 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(WPC_UART_NUM, &uart_config);
    uart_set_pin(WPC_UART_NUM, WPC_SERIAL_TXD, WPC_SERIAL_RXD, WPC_SERIAL_RTS, WPC_SERIAL_CTS);

    uart_driver_install(WPC_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    return 0;
}

/****************************************************************************/
/*                Public method implementation                              */
/****************************************************************************/
int Serial_open(const char * port_name, unsigned long bitrate)
{
    (void) port_name;
    (void) bitrate;

    if (!uart_initialized)
    {
        init_uart();
        uart_initialized = true;
    }

    /* esp_vfs_dev_uart_use_driver(WPC_UART_NUM); */
    // esp_vfs_dev_uart_use_nonblocking(WPC_UART_NUM);

    LOGD("Serial opened\n");
    return 0;
}

int Serial_close()
{
    if (uart_initialized)
    {
        uart_driver_delete(WPC_UART_NUM);
        uart_initialized = false;
    }
    LOGD("Serial closed\n");
    return 0;
}

int Serial_read(unsigned char * c, unsigned int timeout_ms)
{
    return uart_read_bytes(WPC_UART_NUM, c, 1, pdMS_TO_TICKS(timeout_ms));
}

int Serial_write(const unsigned char * buffer, unsigned int buffer_size)
{
    return uart_write_bytes(WPC_UART_NUM, (const char *) buffer, buffer_size);
}
