/*
 * bt_mx01.h
 *
 * Change Logs:
 * Date           Author            Notes
 * 2023-05-06     qiyongzhong       first version
 */

#ifndef __BT_MX01_H__
#define __BT_MX01_H__

#include <rtconfig.h>

//#define BT_MX01_USING_SAMPLE

#define BT_MX01_BAUDRATE            115200  //串口波特率
#define BT_MX01_BYTE_TMO_MS         5       //字节超时
#define BT_MX01_ACK_TMO_MS          500     //应答超时
#define BT_MX01_CMD_INTV_MS         50      //命令间隔
#define BT_MX01_SEND_INTV_MS        50      //数据发送间隔
#define BT_MX01_UAT_BUF_SIZE        256     //AT缓冲区尺寸
#define BT_MX01_MTU                 244     //最大传输单元尺寸
#define BT_MX01_TEST_TIMES          15      //通信测试次数
#define BT_MX01_RETRY_TIMES         3       //重试次数
#define BT_MX01_CONPARAM            0       //连接参数, 0 - 非低功耗，1 - 低功耗

#define BT_MX01_THREAD_STK_SIZE     1024
#define BT_MX01_THREAD_PRIO         7

typedef enum{
    BT_CTRL_RESET = 0,  //复位
    BT_CTRL_SLEEP,      //休眠
    BT_CTRL_WAKEUP,     //唤醒
    BT_CTRL_GET_STATUS, //获取当前状态
    BT_CTRL_SET_NOTIFY, //设置接收通知回调函数
    BT_CTRL_CLR_RECV    //清除接收缓存数据
}bt_ctrl_t;

typedef enum{
    BT_PWR_5dB = 0,     //5dB
    BT_PWR_4dB,         //4dB
    BT_PWR_3dB,         //3dB
    BT_PWR_0dB,         //0dB
    BT_PWR_N2dB,        //-2dB
    BT_PWR_N5dB,        //-5dB
    BT_PWR_N6dB,        //-6dB
    BT_PWR_N10dB,       //-10dB
    BT_PWR_N15dB,       //-15dB
    BT_PWR_N20dB        //-20dB
}bt_power_t;

typedef enum{
    BT_STA_CLOSE = 0,   //关闭
    BT_STA_CFG,         //正在进行配置
    BT_STA_READY,       //启动完成，等待设备连接
    BT_STA_CONNECT,     //已连接
    BT_STA_SLEEP,       //休眠
}bt_status_t;

typedef struct{
    const char *serial;     //串口设备名
    int sleep;              //休眠控制引脚
    int fifo_size;          //FIFO尺寸
    int aintvl;             //广播间隔，20~10000 ms
    bt_power_t power;       //发射功率
}bt_cfg_t;

struct bt_device;
typedef struct bt_device *bt_dev_t;

bt_dev_t bt_create(const char *name, const bt_cfg_t *cfg);//创建蓝牙设备
void bt_destory(bt_dev_t dev);//销毁蓝牙设备
int bt_open(bt_dev_t dev);//打开
int bt_close(bt_dev_t dev);//关闭
int bt_read(bt_dev_t dev, int pos, void *buf, int bufsize);//读取接收数据
int bt_write(bt_dev_t dev, int pos, void *buf, int size);//写入发送数据
int bt_control(bt_dev_t dev, int cmd, void *args);//控制

#endif
