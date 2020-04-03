/* Copyright(c) 2020. Vinetech. All rights reserved. */
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define LOG_MODULE_NAME "esp32_plat"
#define MAX_LOG_LEVEL INFO_LOG_LEVEL
#include "logger.h"
#include "wpc_internal.h"

// Maximum number of indication to be retrieved from a single poll
#define MAX_NUMBER_INDICATION 30

// Polling interval to check for indication
#define POLLING_INTERVAL_MS 20

// Maximum duration in s of failed poll request to declare the link broken
// (unplugged) Set it to 0 to disable 60 sec is long period but it must cover
// the OTAP exchange with neighbors that can be long with some profiles
#define DEFAULT_MAX_POLL_FAIL_DURATION_S 60

// Mutex for sending, ie serial access
static SemaphoreHandle_t sending_mutex;

// This task is used to poll for indication
#define POLLING_TASK_STACKSIZE (4096)
#define POLLING_TASK_PRIORITY 6
static StackType_t polling_task_stack[POLLING_TASK_STACKSIZE];
static StaticTask_t polling_task_tcb;
static TaskHandle_t polling_task_handle;

// This task is used to dispatch indication
#define DISPATCH_TASK_STACKSIZE (4096)
#define DISPATCH_TASK_PRIORITY 5
static StackType_t dispatch_task_stack[DISPATCH_TASK_STACKSIZE];
static StaticTask_t dispatch_task_tcb;
static TaskHandle_t dispatch_task_handle;

// Last successful poll request
static time_t m_last_successful_poll_ts;
static unsigned int m_max_poll_fail_duration_s;

/*****************************************************************************/
/*                Indication queue related variables                        */
/*****************************************************************************/

// Size of the queue between getter and dispatcher of indication
// In most of the cases, the dispatcher is supposed to be faster and the
// queue will have only one element
// But if some execution handling are too long or require to send something over
// UART, the dispatching thread will be hanged until the poll thread finish its
// work
#define MAX_NUMBER_INDICATION_QUEUE (MAX_NUMBER_INDICATION * 2)

// Struct that describes a received frame with its timestamp
typedef struct
{
    wpc_frame_t frame;                      //< The received frame
    unsigned long long timestamp_ms_epoch;  //< The timestamp of reception
} timestamped_frame_t;

// Indications queue
static uint8_t m_indications_queue[sizeof(timestamped_frame_t) * MAX_NUMBER_INDICATION_QUEUE];
static StaticQueue_t indications_queue;
static QueueHandle_t indications_queue_handle;

/*****************************************************************************/
/*                Dispatch indication Thread implementation                  */
/*****************************************************************************/
/**
 * \brief   Thread to dispatch indication in a non locked environment
 */
static void dispatch_indication(void * unused)
{
    while (true)
    {
        static timestamped_frame_t ind = {0};
        while (xQueueReceive(indications_queue_handle, (void *) &ind, portMAX_DELAY))
        {
            // Dispatch the indication
            WPC_Int_dispatch_indication(&ind.frame, ind.timestamp_ms_epoch);
        }
    }
    LOGE("Exiting dispatch thread\n");
}

/*****************************************************************************/
/*                Polling Thread implementation                              */
/*****************************************************************************/
static void onIndicationReceivedLocked(wpc_frame_t * frame, unsigned long long timestamp_ms)
{
    LOGD("Frame received with timestamp = %lld\n", timestamp_ms);

    // Check if queue is full
    if (uxQueueSpacesAvailable(indications_queue_handle) == 0)
    {
        // Queue is FULL
        LOGE("No more room for indications! Must never happen!\n");
    }
    else
    {
        // Insert our received indication
        static timestamped_frame_t ind = {0};
        memcpy(&ind.frame, frame, sizeof(wpc_frame_t));
        ind.timestamp_ms_epoch = timestamp_ms;

        if (xQueueSend(indications_queue_handle, &ind, pdMS_TO_TICKS(500)) != pdTRUE)
        {
            LOGE("sending item to queue is failed\n");
        }
    }
}

/**
 * \brief   Utility function to get current timesatmp in s.
 */
static time_t get_timestamp_s()
{
    struct timespec spec;

    // Get timestamp in ms since epoch
    clock_gettime(CLOCK_REALTIME, &spec);
    return spec.tv_sec;
}

/**
 * \brief   Polling thread.
 *          This thread polls for indication and insert them to the queue
 *          shared with the dispatcher thread
 */
static void poll_for_indication(void * unused)
{
    unsigned int max_num_indication, free_buffer_room;
    int get_ind_res;
    uint32_t wait_before_next_polling_ms = 0;

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(wait_before_next_polling_ms));

        /* Ask for maximum room in buffer queue and less than MAX */
        free_buffer_room = uxQueueSpacesAvailable(indications_queue_handle);
        if (free_buffer_room == 0)
        {
            // Queue is FULL, wait for POLLING INTERVALL to give some
            // time for the dispatching thread to handle them
            LOGW("Queue is full, do not poll (%d)\n", free_buffer_room);
            wait_before_next_polling_ms = POLLING_INTERVAL_MS;
            continue;
        }
        else if (free_buffer_room == MAX_NUMBER_INDICATION_QUEUE)
        {
            // Queue is empty
            free_buffer_room = MAX_NUMBER_INDICATION_QUEUE;
        }

        max_num_indication = free_buffer_room > MAX_NUMBER_INDICATION ?
                                 MAX_NUMBER_INDICATION :
                                 free_buffer_room;

        LOGD("Poll for %d indications\n", max_num_indication);

        get_ind_res = WPC_Int_get_indication(max_num_indication, onIndicationReceivedLocked);

        if (get_ind_res == 1)
        {
            // Still pending indication, only wait 1 ms to give a chance
            // to other threads but not more to have better throughput
            wait_before_next_polling_ms = 1;
        }
        else
        {
            // In case of error or if no more indication, just wait
            // the POLLING INTERVAL to avoid polling all the time
            wait_before_next_polling_ms = POLLING_INTERVAL_MS;
        }

        if (m_max_poll_fail_duration_s > 0)
        {
            if (get_ind_res >= 0 || get_ind_res == WPC_INT_SYNC_ERROR)
            {
                // Poll request executed fine or at least com is working with sink,
                // reset fail counter
                m_last_successful_poll_ts = get_timestamp_s();
            }
            else
            {
                if (get_timestamp_s() - m_last_successful_poll_ts > m_max_poll_fail_duration_s)
                {
                    // Poll request has failed for too long, just exit
                    break;
                }
            }
        }
    }

    LOGE("Exiting polling thread\n");
}

void Platform_usleep(unsigned int time_us)
{
    usleep(time_us);
}

bool Platform_lock_request()
{
    while (xSemaphoreTake(sending_mutex, portMAX_DELAY) != pdPASS)
    {
        // It must never happen but add a check and
        // return to avoid a deadlock
        LOGE("Mutex already locked\n");
        return false;
    }

    return true;
}

void Platform_unlock_request()
{
    xSemaphoreGive(sending_mutex);
}

unsigned long long Platform_get_timestamp_ms_epoch()
{
    struct timespec spec;

    // Get timestamp in ms since epoch
    clock_gettime(CLOCK_REALTIME, &spec);
    return ((unsigned long long) spec.tv_sec) * 1000 + (spec.tv_nsec) / 1000 / 1000;
}

bool Platform_init()
{
    // Initialize mutex to access critical section
    sending_mutex = xSemaphoreCreateMutex();
    if (!sending_mutex)
    {
        LOGE("Sending Mutex init failed\n");
        goto error1;
    }

    indications_queue_handle = xQueueCreateStatic(MAX_NUMBER_INDICATION_QUEUE,
                                                  sizeof(timestamped_frame_t),
                                                  m_indications_queue,
                                                  &indications_queue);

    // Start a task to poll for indication
    polling_task_handle = xTaskCreateStaticPinnedToCore(poll_for_indication,
                                                        "polling_task",
                                                        POLLING_TASK_STACKSIZE,
                                                        NULL,
                                                        tskIDLE_PRIORITY,
                                                        polling_task_stack,
                                                        &polling_task_tcb,
                                                        1);

    // Start a thread to dispatch indication
    dispatch_task_handle = xTaskCreateStaticPinnedToCore(dispatch_indication,
                                                         "dispatch_task",
                                                         DISPATCH_TASK_STACKSIZE,
                                                         NULL,
                                                         tskIDLE_PRIORITY,
                                                         dispatch_task_stack,
                                                         &dispatch_task_tcb,
                                                         1);

    m_last_successful_poll_ts = get_timestamp_s();
    m_max_poll_fail_duration_s = DEFAULT_MAX_POLL_FAIL_DURATION_S;

    return true;

error1:
    return false;
}

bool Platform_set_max_poll_fail_duration(unsigned long duration_s)
{
    if ((m_max_poll_fail_duration_s == 0) && (duration_s > 0))
    {
        m_last_successful_poll_ts = get_timestamp_s();
    }
    m_max_poll_fail_duration_s = duration_s;
    return true;
}

void Platform_close()
{
    vTaskDelete(polling_task_handle);
    vTaskDelete(dispatch_task_handle);
}
