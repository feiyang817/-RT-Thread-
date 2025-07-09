/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-12-10     Rbb666        first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "hal_data.h"
#include "ra/board/ra8d1_ek/board_sdram.h"
#ifdef RT_USING_DFS
    #include <unistd.h>
    #include <fcntl.h>
#endif
#ifdef BSP_USING_OPENMV
    #include <py/compile.h>
    #include <py/runtime.h>
    #include <py/repl.h>
    #include <py/gc.h>
    #include <py/mperrno.h>
    #include <py/stackctrl.h>
    #include <py/frozenmod.h>
    #include <py/mphal.h>
    #include <lib/utils/pyexec.h>
    #include <lib/mp-readline/readline.h>
    #include <framebuffer.h>
    #include <fb_alloc.h>
    #include <ff_wrapper.h>
    #include <usbdbg.h>
    #include <sensor.h>
    #include <tinyusb_debug.h>
    #include <tusb.h>
    #include <led.h>
    #include <mpy_main.h>
//不使用软件包的情况下
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

//#define DBG_TAG "main"
//#define DBG_LVL         DBG_LOG
//#include <rtdbg.h>
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG

#endif /* BSP_USING_OPENMV */
#ifdef RT_USING_FAL
    #include "fal.h"
#endif /* RT_USING_FAL */

#define DRV_DEBUG
#define LOG_TAG             "main"
#define ADC_DEV_NAME        "adc0"      /* ADC 设备名称 */
#define ADC_DEV_CHANNEL     0           /* ADC 通道 */
#define REFER_VOLTAGE       330         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */
#define LED_PIN    BSP_IO_PORT_01_PIN_02 /* Onboard LED pins */
#define GPIO_PIN001  BSP_IO_PORT_00_PIN_01
#define GPIO_PIN507  BSP_IO_PORT_05_PIN_07
#define GPIO_PIN508  BSP_IO_PORT_05_PIN_08
#define GPIO_PIN509  BSP_IO_PORT_05_PIN_09
#define GPIO_PIN510  BSP_IO_PORT_05_PIN_10

#define AHT_I2C_BUS_NAME    "i2c0"  //AHT20 挂载的I2C总线
#define AHT_ADDR            0x38    //AHT20 I2C地址
#define AHT_CALIBRATION_CMD 0xBE    //AHT20 初始化命令
#define AHT_NORMAL_CMD      0xA8    //AHT20 正常工作模式命令
#define AHT_GET_DATA_CMD    0xAC    //AHT20 获取结果命令

#include <drv_log.h>

/* MicroPython runs as a task under RT-Thread */
#define MP_TASK_STACK_SIZE      (64 * 1024)

#ifdef BSP_USING_OPENMV
static void *stack_top = RT_NULL;
static char OMV_ATTR_SECTION(OMV_ATTR_ALIGNED(gc_heap[OMV_HEAP_SIZE], 4), ".sdram");

extern int mount_init(void);
extern void fmath_init(void);
extern int tusb_board_init(void);
static bool exec_boot_script(const char *path, bool interruptible);

void *__signgam_addr(void)
{
    return NULL;
}

void flash_error(int n)
{
    led_state(LED_RED, 0);
    led_state(LED_GREEN, 0);
    led_state(LED_BLUE, 0);
    for (int i = 0; i < n; i++)
    {
        led_state(LED_RED, 0);
        rt_thread_mdelay(100);
        led_state(LED_RED, 1);
        rt_thread_mdelay(100);
    }
    led_state(LED_RED, 0);
}

void NORETURN __fatal_error(const char *msg)
{
    rt_thread_mdelay(100);
    led_state(1, 1);
    led_state(2, 1);
    led_state(3, 1);
    led_state(4, 1);
    mp_hal_stdout_tx_strn("\nFATAL ERROR:\n", 14);
    mp_hal_stdout_tx_strn(msg, strlen(msg));
    for (uint i = 0;;)
    {
        led_toggle(((i++) & 3) + 1);
        rt_thread_mdelay(100);
        if (i >= 16)
        {
            /* to conserve power */
            __WFI();
        }
    }
}

static void omv_entry(void *parameter)
{
    (void) parameter;
    int stack_dummy;
    stack_top = (void *)&stack_dummy;

    bool first_soft_reset = true;

#ifdef __DCACHE_PRESENT
    /* Clean and enable cache */
    SCB_CleanDCache();
#endif
    tusb_board_init();
#ifdef RT_USING_FAL
    fal_init();
#endif
#ifdef BSP_USING_FS
    /* wait sdcard mount */
    extern struct rt_semaphore sem_mnt_lock;;
    rt_sem_take(&sem_mnt_lock, 400);

    struct dfs_fdtable *fd_table_bak = NULL;
#endif
    fmath_init();
#if MICROPY_PY_THREAD
    mp_thread_init(rt_thread_self()->stack_addr, MP_TASK_STACK_SIZE / sizeof(uintptr_t));
#endif
soft_reset:
#ifdef BSP_USING_FS
    mp_sys_resource_bak(&fd_table_bak);
#endif
    led_state(LED_RED, 1);
    led_state(LED_GREEN, 1);
    led_state(LED_BLUE, 1);

    fb_alloc_init0();
    framebuffer_init0();

    /* Stack limit should be less than real stack size, so we have a */
    /* chance to recover from limit hit. (Limit is measured in bytes) */
    mp_stack_set_top(stack_top);
    mp_stack_set_limit(MP_TASK_STACK_SIZE - 1024);

    /* GC init */
    gc_init(&gc_heap[0], &gc_heap[MP_ARRAY_SIZE(gc_heap)]);

    /* MicroPython initialization */
    mp_init();

    readline_init0();
    imlib_init_all();

    usb_cdc_init();
    usbdbg_init();
    file_buffer_init0();
    sensor_init0();

#if MICROPY_PY_SENSOR
    if (sensor_init() != 0 && first_soft_reset)
    {
        LOG_E("sensor init failed!");
    }
#endif

    /* run boot.py and main.py if they exist */
    bool interrupted = exec_boot_script("boot.py", false);

    /* urn boot-up LEDs off */
    led_state(LED_RED, 0);
    led_state(LED_GREEN, 0);
    led_state(LED_BLUE, 0);

    /* Run main.py script on first soft-reset */
    if (first_soft_reset && !interrupted)
    {
        exec_boot_script("main.py", true);
        goto soft_reset_exit;
    }

    /* If there's no script ready, just re-exec REPL */
    while (!usbdbg_script_ready())
    {
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0)
        {
            /* enable IDE interrupt */
            usbdbg_set_irq_enabled(true);
            /* run REPL */
            if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL)
            {
                if (pyexec_raw_repl() != 0)
                {
                    break;
                }
            }
            else
            {
                if (pyexec_friendly_repl() != 0)
                {
                    break;
                }
            }
            nlr_pop();
        }
    }

    LOG_I("Exit from repy!");

    if (usbdbg_script_ready())
    {
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0)
        {
            /* Enable IDE interrupt */
            usbdbg_set_irq_enabled(true);
            /* Execute the script */
            pyexec_str(usbdbg_get_script());
            /* Disable IDE interrupts */
            usbdbg_set_irq_enabled(false);
            nlr_pop();
        }
        else
        {
            mp_obj_print_exception(&mp_plat_print, (mp_obj_t) nlr.ret_val);
        }

        if (usbdbg_is_busy() && nlr_push(&nlr) == 0)
        {
            /* Enable IDE interrupt */
            usbdbg_set_irq_enabled(true);
            /* Wait for the current command to finish */
            usbdbg_wait_for_command(1000);
            /* Disable IDE interrupts */
            usbdbg_set_irq_enabled(false);
            nlr_pop();
        }
    }

soft_reset_exit:
    /* soft reset */
    mp_printf(&mp_plat_print, "MPY: soft reboot\n");

    gc_sweep_all();
    mp_deinit();
#if MICROPY_PY_THREAD
    mp_thread_deinit();
#endif
#ifdef RT_USING_DFS
    mp_sys_resource_gc(fd_table_bak);
#endif
#ifdef OPENMV_USING_KEY
    rt_uint32_t pin = rt_pin_get(USER_KEY_PIN_NAME);
    rt_pin_detach_irq(pin);
#endif
    first_soft_reset = false;
    goto soft_reset;
}

static bool exec_boot_script(const char *path, bool interruptible)
{
    nlr_buf_t nlr;
    bool interrupted = false;
    if (nlr_push(&nlr) == 0)
    {
        /* Enable IDE interrupts if allowed */
        if (interruptible)
        {
            usbdbg_set_irq_enabled(true);
            usbdbg_set_script_running(true);
        }

        /* Parse, compile and execute the script */
        pyexec_file_if_exists(path);
        nlr_pop();
    }
    else
    {
        interrupted = true;
    }

    /* Disable IDE interrupts */
    usbdbg_set_irq_enabled(false);
    usbdbg_set_script_running(false);

    if (interrupted)
    {
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t) nlr.ret_val);
    }

    return interrupted;
}

static void omv_init_func(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("omv", omv_entry, RT_NULL,
                           MP_TASK_STACK_SIZE / sizeof(uint32_t), 22, 20);
    RT_ASSERT(tid != RT_NULL);

    rt_thread_startup(tid);
}
#endif  /* BSP_USING_OPENMV */

/*读取光敏传感器和麦克风传感器的模拟信号，如果都为高电平就使引脚P510输出高电平*/
static void gpio_logic_loop(void *parameter)
{
    rt_int8_t level1, level2;

    /* 1. 配置模式 */
    rt_pin_mode(GPIO_PIN508, PIN_MODE_INPUT);            /* 浮空输入 */
    rt_pin_mode(GPIO_PIN509, PIN_MODE_INPUT_PULLUP);     /* 内部上拉输入（按需） */
    rt_pin_mode(GPIO_PIN510, PIN_MODE_OUTPUT);           /* 推挽输出 */

    while (1)
    {
        /* 2. 同步读取两个输入引脚 */
        level1 = rt_pin_read(GPIO_PIN508);
        level2 = rt_pin_read(GPIO_PIN509);

        /* 3. 如果两个都为高，则输出高；否则输出低 */
        if (level1 == PIN_HIGH && level2 == PIN_HIGH)
        {
            rt_pin_write(GPIO_PIN510, PIN_HIGH);
            rt_thread_mdelay(10000);
        }
        else
        {
            rt_pin_write(GPIO_PIN510, PIN_LOW);
        }

        rt_thread_mdelay(50);  /* 50ms 轮询一次 */
    }
}

/*通过ADC通道读取烟雾传感器的数字信号，当读取的数字信号大于设定值时，引脚P507输出高电平*/
static void adc_vol_sample(void *parameter)
{
    rt_adc_device_t adc_dev;
    rt_uint32_t value, vol;
    rt_err_t ret = RT_EOK;
    rt_int32_t  ppm,a;

    /* 查找设备 */
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return;
    }
    if (ret != RT_EOK)
     {
         rt_kprintf("adc enable failed!\n");
         return;
     }
    /* 使能设备 */
    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);
    while(1)
    {
        value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
        rt_kprintf("the concentration is :%d \n", value);//输出通过ADC通道读取的烟雾传感器的值

        /* 转换为对应电压值 */
        vol = value * REFER_VOLTAGE / CONVERT_BITS;

//        rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);
/*当烟雾传感器读取的值大于设定值时引脚P507输出高电平*/
        if (value > 2500)
               {
                   rt_pin_write(GPIO_PIN507, PIN_HIGH);
                   rt_thread_mdelay(5000);
               }
               else
               {
                   rt_pin_write(GPIO_PIN507, PIN_LOW);
               }
        rt_thread_delay(1000);  /* 每次延时 1000 毫秒 */
    }
    /* 关闭通道 */
    ret = rt_adc_disable(adc_dev, ADC_DEV_CHANNEL);
}

/*通过i2c总线，读取温湿度传感器的数据，当读取到的数据超过设定值时，引脚P001输出高电平*/
static struct rt_i2c_bus_device *i2c_bus = RT_NULL;
static rt_uint8_t Parm_Null[2] = {0, 0};
// 写命令
static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t cmd, rt_uint8_t *data)
{
    rt_uint8_t buf[3];
    rt_uint8_t size;
    struct rt_i2c_msg msg;

    buf[0] = cmd;
    if (data)
    {
        buf[1] = data[0];
        buf[2] = data[1];
        size = 3;
    }
    else
    {
        size = 1;
    }

    msg.addr  = AHT_ADDR;
    msg.flags = RT_I2C_WR;
    msg.buf   = buf;
    msg.len   = size;

    return (rt_i2c_transfer(bus, &msg, 1) == 1) ? RT_EOK : RT_ERROR;
}

// 读数据
static rt_err_t read_reg(struct rt_i2c_bus_device *bus, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msg;

    msg.addr  = AHT_ADDR;
    msg.flags = RT_I2C_RD;
    msg.buf   = buf;
    msg.len   = len;

    return (rt_i2c_transfer(bus, &msg, 1) == 1) ? RT_EOK : RT_ERROR;
}

// 读取温湿度并计算
static void read_temp_humi(float *temperature, float *humidity)
{
    rt_uint8_t data[6];

    write_reg(i2c_bus, AHT_GET_DATA_CMD, Parm_Null);
    rt_thread_mdelay(500);
    read_reg(i2c_bus, 6, data);

    // 湿度计算
    *humidity = ((data[1] << 12) | (data[2] << 4) | ((data[3] & 0xF0) >> 4)) * 100.0f / (1 << 20);
    // 温度计算
    *temperature = (((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) * 200.0f / (1 << 20) - 50;
}

// AHT10 初始化
static void AHT_Init(const char *bus_name)
{
    rt_uint8_t calib_cmd[2] = {0x08, 0x00};

    i2c_bus = rt_i2c_bus_device_find(bus_name);
    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("ERROR: Cannot find I2C bus %s\n", bus_name);
        return;
    }

    // 正常模式
    write_reg(i2c_bus, AHT_NORMAL_CMD, Parm_Null);
    rt_thread_mdelay(400);

    // 校准命令
    write_reg(i2c_bus, AHT_CALIBRATION_CMD, calib_cmd);
    rt_thread_mdelay(400);
}

// 线程参数
#define THREAD_STACK_SIZE  512
#define THREAD_PRIORITY    10
#define THREAD_TIMESLICE   20

// 线程入口函数
static void aht10_thread_entry(void *parameter)
{
    float temperature = 0.0f, humidity = 0.0f;

    AHT_Init(AHT_I2C_BUS_NAME);
    while (1)
    {
        read_temp_humi(&temperature, &humidity);
        rt_kprintf("humidity   : %d.%d %%\n", (int)humidity, (int)(humidity * 10) % 10);//串口输出湿度数据
        rt_kprintf("temperature: %d.%d C\n", (int)temperature, (int)(temperature * 10) % 10);//串口输出温度数据
/*当温度或湿度大于设定值时，引脚P001输出高电平*/
        if (temperature > 30.0f||humidity>80.0f)
               {
                   rt_pin_write(GPIO_PIN001, PIN_HIGH);
                   rt_thread_mdelay(5000);
               }
               else
               {
                   rt_pin_write(GPIO_PIN001, PIN_LOW);
               }
        rt_thread_mdelay(1000);
    }
}

// 自动启动函数
static int aht10_auto_start(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("aht10",
                           aht10_thread_entry,
                           RT_NULL,
                           THREAD_STACK_SIZE,
                           THREAD_PRIORITY,
                           THREAD_TIMESLICE);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    else
        rt_kprintf("ERROR: create aht10 thread failed\n");

    return 0;
}




void hal_entry(void)
{
    rt_thread_t adc_thread;
    /* 创建一个新的线程来周期性采样 ADC */
    adc_thread = rt_thread_create("adc_sample", adc_vol_sample, RT_NULL, 1024, 10, 10);
    if (adc_thread != RT_NULL)
    {
        rt_thread_startup(adc_thread);
    }
    rt_thread_t tid;

    tid = rt_thread_create("gpio_logic", gpio_logic_loop, RT_NULL, 1024, 10, 10);
    if (tid)
        rt_thread_startup(tid);

    LOG_I("===================================");
    LOG_I("This is OpenMV4.1.0 demo");
    LOG_I("===================================");

#ifdef BSP_USING_OPENMV
    omv_init_func();


#endif
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(adc_vol_sample, adc voltage convert sample);
INIT_APP_EXPORT(aht10_auto_start);
