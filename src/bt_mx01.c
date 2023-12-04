/*
 * bt_mx01.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2023-05-06     qiyongzhong       first version
 */

#include <bt_mx01.h>
#include <rtdevice.h>
#include <uat.h>
#include <stddef.h>
#include <string.h>

#define DBG_TAG "bt.mx01"
#define DBG_LVL DBG_LOG //DBG_INFO  //
#include <rtdbg.h>

struct bt_device{
    struct rt_device parent;//设备数据
    uat_inst_t *uat;        //uat instance
    const bt_cfg_t *cfg;    //配置参数指针
    struct rt_ringbuffer *rx_rb;
    bt_status_t status;     //工作状态
};

static void bt_delay_ms(int ms)
{
    if (ms > 0)
    {
        rt_thread_mdelay(ms);
    }
}

static void bt_pins_init(bt_dev_t dev)
{
    if (dev->cfg->sleep >= 0)
    {
        rt_pin_mode(dev->cfg->sleep, PIN_MODE_OUTPUT);
        rt_pin_write(dev->cfg->sleep, PIN_HIGH);
    }
}

static void bt_pins_deinit(bt_dev_t dev)
{
    if (dev->cfg->sleep >= 0)
    {
        rt_pin_mode(dev->cfg->sleep, PIN_MODE_INPUT);
    }
}

static int bt_pin_set_sleep(bt_dev_t dev, int enable)
{
    if (dev->cfg->sleep < 0)
    {
        LOG_E("BT does not support sleep.");
        return(-RT_ERROR);
    }
    
    rt_pin_write(dev->cfg->sleep, (enable ? PIN_HIGH : PIN_LOW));
    return(RT_EOK);
}

static void bt_set_end_sign(bt_dev_t dev)
{
    uat_set_end_sign(dev->uat, "\r\n");
}

static void bt_clr_end_sign(bt_dev_t dev)
{
    uat_set_end_sign(dev->uat, NULL);
}

//复位芯片
static int bt_cmd_reset(bt_dev_t dev)
{
    int len = uat_execute_at_cmd(dev->uat, BT_MX01_ACK_TMO_MS, "AT+REBOOT=1");
    if (len <= 0)
    {
        return(-RT_ERROR);
    }
    return(RT_EOK);
}

//设置蓝牙功率：范围 0-9；重启后生效。
static int bt_cmd_set_power(bt_dev_t dev, int power)
{
    int len = uat_execute_at_cmd(dev->uat, BT_MX01_ACK_TMO_MS, "AT+TXPOWER=%d", power);
    if (len <= 0)
    {
        return(-RT_ERROR);
    }
    return(RT_EOK);
}

//设置广播间隔：范围 20-10000 ms，重启后生效。
static int bt_cmd_set_aintvl(bt_dev_t dev, int aintvl)
{
    int len = uat_execute_at_cmd(dev->uat, BT_MX01_ACK_TMO_MS, "AT+AINTVL=%d", aintvl);
    if (len <= 0)
    {
        return(-RT_ERROR);
    }
    return(RT_EOK);
}

//设置蓝牙名字，名字最长20个字节
static int bt_cmd_set_name(bt_dev_t dev, const char *name)
{
    int len = uat_execute_at_cmd(dev->uat, BT_MX01_ACK_TMO_MS, "AT+NAME=%s", name);
    if (len <= 0)
    {
        return(-RT_ERROR);
    }
    return(RT_EOK);
}

//查询蓝牙连接, <0 - 错误，0 - 未连接，1 - 已连接
static int bt_cmd_chk_link(bt_dev_t dev)
{
    bt_set_end_sign(dev);
    int len = uat_execute_at_cmd(dev->uat, BT_MX01_ACK_TMO_MS, "AT+DEV?");
    bt_clr_end_sign(dev);
    if (len <= 0)
    {
        return(-RT_ERROR);
    }
    if (uat_search_keyword(uat_get_buf(dev->uat), len, "+DEV:1") != RT_NULL)
    {
        return(1);
    }
    return(RT_EOK);
}

static int bt_wait_comm_ok(bt_dev_t dev)//等待通信正常
{
    for (int i=0; i<BT_MX01_TEST_TIMES; i++)
    {
        bt_delay_ms(1000);
        if (bt_cmd_chk_link(dev) >= 0)
        {
            LOG_D("BT communication success.");
            return(RT_EOK);
        }
    }
    LOG_E("BT communication fail.");
    return(-RT_ERROR);
}

static int bt_reset_chip(bt_dev_t dev)//复位芯片
{
    for (int i=0; i<BT_MX01_RETRY_TIMES; i++)
    {
        bt_delay_ms(BT_MX01_CMD_INTV_MS); 
        if (bt_cmd_reset(dev) == RT_EOK)
        {
            LOG_D("BT reset chip success.");
            return(RT_EOK);
        }
    }
    LOG_E("BT reset chip fail.");
    return(-RT_ERROR);
}

static int bt_cfg_power(bt_dev_t dev)//配置功率
{
    int power = dev->cfg->power;
    for (int i=0; i<BT_MX01_RETRY_TIMES; i++)
    {
        bt_delay_ms(BT_MX01_CMD_INTV_MS);
        if (bt_cmd_set_power(dev, power) == RT_EOK)
        {
            LOG_D("BT config power=%d success.", power);
            return(RT_EOK);
        }
    }
    LOG_E("BT config power=%d fail.", power);
    return(-RT_ERROR);
}

static int bt_cfg_aintvl(bt_dev_t dev)//配置广播间隔
{
    int aintvl = dev->cfg->aintvl;
    for (int i=0; i<BT_MX01_RETRY_TIMES; i++)
    {
        bt_delay_ms(BT_MX01_CMD_INTV_MS);
        if (bt_cmd_set_aintvl(dev, aintvl) == RT_EOK)
        {
            LOG_D("BT config aintvl=%d success.", aintvl);
            return(RT_EOK);
        }
    }
    LOG_E("BT config aintvl=%d fail.", aintvl);
    return(-RT_ERROR);
}

static int bt_cfg_name(bt_dev_t dev)//配置名字
{
    char name[24];
    strncpy(name, dev->parent.parent.name, RT_NAME_MAX);
    for (int i=0; i<BT_MX01_RETRY_TIMES; i++)
    {
        bt_delay_ms(BT_MX01_CMD_INTV_MS);
        if (bt_cmd_set_name(dev, (void *)name) == RT_EOK)
        {
            LOG_D("BT config name success. name = %s", name);
            return(RT_EOK);
        }
    }
    LOG_E("BT config name fail.");
    return(-RT_ERROR);
}

static void bt_fsm_cfg_deal(bt_dev_t dev)
{
    if (bt_wait_comm_ok(dev) != RT_EOK)
    {
        return;
    }
    if (bt_cfg_power(dev) != RT_EOK)
    {
        return;
    }
    if (bt_cfg_aintvl(dev) != RT_EOK)
    {
        return;
    }
    if (bt_reset_chip(dev) != RT_EOK)
    {
        return;
    }
    bt_delay_ms(1000);
    if (bt_cfg_name(dev) != RT_EOK)
    {
        return;
    }
    dev->status = BT_STA_READY;
    LOG_D("BT config ok...");
}

static void bt_fsm_ready_deal(bt_dev_t dev)
{
    if (bt_cmd_chk_link(dev) != 1)
    {
        rt_uint32_t ticks = rt_tick_get();
        while(1)
        {
            char *buf = uat_get_buf(dev->uat);
            int len = uat_data_recv(dev->uat, buf, BT_MX01_UAT_BUF_SIZE, 1000);
            if ((len > 0) && (uat_search_keyword(buf, len, "+CONNECTED:1") != RT_NULL))
            {
                break;
            }
            if (rt_tick_get() - ticks >= rt_tick_from_millisecond(3 * 1000))
            {
                return;
            }
        }
    }
    dev->status = BT_STA_CONNECT;
    LOG_D("BT connect...");
}

static void bt_fsm_connect_deal(bt_dev_t dev)
{
    char *buf = uat_get_buf(dev->uat);
    int len = uat_data_recv(dev->uat, buf, BT_MX01_UAT_BUF_SIZE, (3 * 1000));
    if (len <= 0)
    {
        return;
    }
    if (uat_search_keyword(buf, len, "+DISCONN:1") != RT_NULL)
    {
        dev->status = BT_STA_READY;
        LOG_D("BT disconnect...");
        return;
    }
    rt_ringbuffer_put(dev->rx_rb, (void *)buf, len);
    if (dev->parent.rx_indicate)
    {
        dev->parent.rx_indicate((rt_device_t)dev, rt_ringbuffer_data_len(dev->rx_rb));
    }
}

static void bt_fsm_sleep_deal(bt_dev_t dev)
{
    bt_delay_ms(10);
}

static void bt_thread_entry(void *args)
{
    bt_dev_t dev = (bt_dev_t)args;
    
    bt_delay_ms(200);
    while(1)
    {
        switch(dev->status)
        {
        case BT_STA_CFG:
            bt_fsm_cfg_deal(dev);
            break;
        case BT_STA_READY:
            bt_fsm_ready_deal(dev);
            break;
        case BT_STA_CONNECT:
            bt_fsm_connect_deal(dev);
            break;
        case BT_STA_SLEEP:
            bt_fsm_sleep_deal(dev);
        default:
            return;
        }
    }
}

static int bt_thread_create(bt_dev_t dev)
{
    rt_thread_t tid = rt_thread_create(dev->parent.parent.name, bt_thread_entry, dev, 
                                        BT_MX01_THREAD_STK_SIZE, BT_MX01_THREAD_PRIO, 20);
    if (tid == RT_NULL)
    {
        LOG_E("BT create thread fail.");
        return(-RT_ERROR);
    }
    
    rt_thread_startup(tid);

    LOG_D("BT create thread success.");
    return(RT_EOK);
}

static int bt_reset(bt_dev_t dev)
{
    if (dev->status == BT_STA_CLOSE)
    {
        LOG_E("BT reset fail. it is closed.");
        return(-RT_ERROR);
    }
    
    bt_pin_set_sleep(dev, 0);
    dev->status = BT_STA_CFG;

    LOG_E("BT reset success.");
    return(RT_EOK);
}

static int bt_sleep(bt_dev_t dev)
{
    if (dev->status < BT_STA_READY)
    {
        LOG_E("BT sleep fail. it is not ready.");
        return(-RT_ERROR);
    }
    
    int rst = bt_pin_set_sleep(dev, 0);
    if (rst == RT_EOK)
    {
        dev->status = BT_STA_SLEEP;
    }
    
    return(rst);
}

static int bt_wackup(bt_dev_t dev)
{
    if (dev->status != BT_STA_SLEEP)
    {
        LOG_D("BT wakeup fail. it is not sleeping.");
        return(RT_EOK);
    }
    
    int rst = bt_pin_set_sleep(dev, 0);
    if (rst == RT_EOK)
    {
        LOG_D("BT wakeup success.");
        dev->status = BT_STA_READY;
    }

    return(rst);
}

static bt_status_t bt_get_status(bt_dev_t dev)
{
    return(dev->status);
}

static int bt_set_notify(bt_dev_t dev, rt_err_t(*notify)(rt_device_t dev, rt_size_t size))
{
    return(rt_device_set_rx_indicate((rt_device_t)dev, notify));
}


#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops bt_ops =
{
    RT_NULL,
    (void*)bt_open,
    (void*)bt_close,
    (void*)bt_read,
    (void*)bt_write,
    (void*)bt_control
};
#endif

bt_dev_t bt_create(const char *name, const bt_cfg_t *cfg)//创建蓝牙设备
{
    RT_ASSERT(name != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    RT_ASSERT(cfg->serial != RT_NULL);

    rt_device_t device = rt_device_create(RT_Device_Class_Char, (sizeof(struct bt_device) - sizeof(struct rt_device)));
    if (device == RT_NULL)
    {
        LOG_E("BT create device fail.");
        return(RT_NULL);
    }

    bt_dev_t dev = (bt_dev_t)device;
    dev->uat = uat_inst_create(cfg->serial, BT_MX01_BAUDRATE, BT_MX01_BYTE_TMO_MS, BT_MX01_UAT_BUF_SIZE);
    if (dev->uat == RT_NULL)
    {
        bt_destory(dev);
        LOG_E("BT create uat instance fail.");
        return(RT_NULL);
    }

    dev->rx_rb = rt_ringbuffer_create(cfg->fifo_size);
    if (dev->rx_rb == RT_NULL)
    {
        bt_destory(dev);
        LOG_E("BT create ringbuffer fail.\n");
        return(RT_NULL);
    }
    
    dev->cfg = cfg;
    dev->status = BT_STA_CLOSE;

    bt_pins_init(dev);

#ifdef RT_USING_DEVICE_OPS
    device->ops     = &bt_ops;
#else
    device->init    = RT_NULL;
    device->open    = (void*)bt_open;
    device->close   = (void*)bt_close;
    device->read    = (void*)bt_read;
    device->write   = (void*)bt_write;
    device->control = (void*)bt_control;
#endif

    rt_device_register((void *)dev, name, RT_DEVICE_FLAG_RDWR);

    LOG_D("BT create device success.\n");

    return(dev);
}

void bt_destory(bt_dev_t dev)//销毁蓝牙设备
{
    RT_ASSERT(dev != RT_NULL);

    bt_close(dev);
    bt_pins_deinit(dev);

    rt_device_unregister((void *)dev);

    if (dev->rx_rb != RT_NULL)
    {
        rt_ringbuffer_destroy(dev->rx_rb);
        dev->rx_rb = RT_NULL;
    }

    if (dev->uat != RT_NULL)
    {
        uat_inst_destory(dev->uat);
        dev->uat = RT_NULL;
    }

    rt_device_destroy((void *)dev);

    LOG_D("BT destory device success.\n");
}

int bt_open(bt_dev_t dev)//打开
{
    RT_ASSERT(dev != RT_NULL);
    
    if (dev->status != BT_STA_CLOSE)//已打开
    {
        LOG_D("BT has been opened.");
        return(RT_EOK);
    }
    
    rt_thread_t tid = rt_thread_find(dev->parent.parent.name);
    if (tid != RT_NULL)
    {
        LOG_D("BT thread already exists.");
        return(RT_EOK);
    }
    
    dev->status = BT_STA_CFG;
    bt_pin_set_sleep(dev, 0);
    if (bt_thread_create(dev) != RT_EOK)
    {
        return(-RT_ERROR);
    }

    LOG_D("BT open success.");
    return(RT_EOK);
}

int bt_close(bt_dev_t dev)//关闭
{
    RT_ASSERT(dev != RT_NULL);
    
    if (dev->status == BT_STA_CLOSE)
    {
        LOG_D("BT has been closed.");
        return(RT_EOK);
    }
    
    rt_thread_t tid = rt_thread_find(dev->parent.parent.name);
    if (tid != RT_NULL)
    {
        rt_thread_delete(tid);
    }

    bt_pin_set_sleep(dev, 1);
    dev->status = BT_STA_CLOSE;

    LOG_D("BT close success.");
    return(RT_EOK);
}

int bt_read(bt_dev_t dev, int pos, void *buf, int bufsize)//读取接收数据
{
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buf != RT_NULL);
    RT_ASSERT(bufsize > 0);
    
    if (dev->status != BT_STA_CONNECT)
    {
        LOG_D("BT read fail. it is not connected.");
        return(-RT_ERROR);
    }
    return(rt_ringbuffer_get(dev->rx_rb, buf, bufsize));
}

int bt_write(bt_dev_t dev, int pos, void *buf, int size)//写入发送数据
{
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buf != RT_NULL);
    RT_ASSERT(size > 0);
    
    if (dev->status != BT_STA_CONNECT)
    {
        LOG_D("BT write fail. it is not connected.");
        return(-RT_ERROR);
    }
    pos = 0;
    while(pos < size)
    {
        int slen = size - pos;
        if (slen > BT_MX01_MTU)
        {
            slen = BT_MX01_MTU;
        }
        slen = uat_data_send(dev->uat, buf + pos, slen);
        if (slen <= 0)
        {
            return(-RT_ERROR);
        }
        if (slen == BT_MX01_MTU)
        {
            bt_delay_ms(BT_MX01_SEND_INTV_MS);
        }
        pos += slen;
    }
    if (dev->parent.tx_complete)
    {
        (dev->parent.tx_complete)((void *)dev, buf);
    }
    return(size);
}

int bt_control(bt_dev_t dev, int cmd, void *args)//控制
{
    RT_ASSERT(dev != RT_NULL);

    int rst = RT_EOK;
    switch(cmd)
    {
    case BT_CTRL_RESET:
        rst = bt_reset(dev);
        break;
    case BT_CTRL_SLEEP:
        rst = bt_sleep(dev);
        break;
    case BT_CTRL_WAKEUP:
        rst = bt_wackup(dev);
        break;
    case BT_CTRL_GET_STATUS:
        *((bt_status_t *)args) = bt_get_status(dev);
        break;
    case BT_CTRL_SET_NOTIFY:
        rst = bt_set_notify(dev, args);
        break;
    case BT_CTRL_CLR_RECV:
        rt_ringbuffer_reset((struct rt_ringbuffer *)(dev->rx_rb));
        break;
    default:
        rst = -RT_ERROR;
        LOG_E("command (%d) does not support", cmd);
        break;
    }

    return(rst);
}

