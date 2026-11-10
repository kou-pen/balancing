#include "balancing/device/balancing_f446_motor_bench_port.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

static CAN_HandleTypeDef can1 = {1u};
static CAN_HandleTypeDef can2 = {2u};
typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    unsigned tx_count;
    bool rx_pending;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
} TestCan;
static TestCan test_can1;
static TestCan test_can2;
static uint32_t irq_mask;
static TestCan *mock(CAN_HandleTypeDef *h)
{
    return (h == &can1) ? &test_can1 : &test_can2;
}

uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1u; }
void __enable_irq(void) { irq_mask = 0u; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    return 1u;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *header,
                                      uint8_t *data, uint32_t *mailbox)
{
    TestCan *mock_can = mock(hcan);
    mock_can->header = *header;
    memcpy(mock_can->data, data, 8);
    mock_can->tx_count++;
    *mailbox = CAN_TX_MAILBOX0;
    return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan, uint32_t fifo)
{
    assert(fifo == CAN_RX_FIFO0);
    return mock(hcan)->rx_pending ? 1u : 0u;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo,
                                      CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    TestCan *mock_can = mock(hcan);
    assert(fifo == CAN_RX_FIFO0);
    if (!mock_can->rx_pending) return HAL_ERROR;
    *header = mock_can->rx_header;
    memcpy(data, mock_can->rx_data, 8);
    mock_can->rx_pending = false;
    return HAL_OK;
}
static void rx_rs_good(uint64_t stamp_us)
{
    test_can1.rx_header = (CAN_RxHeaderTypeDef){
        .IDE=CAN_ID_EXT, .RTR=CAN_RTR_DATA, .DLC=8u,
        .ExtId=0x028001fdu
    };
    const uint8_t frame[8]={0x80,0,0x80,0,0x80,0,1,44};
    memcpy(test_can1.rx_data,frame,8);
    test_can1.rx_pending=true;
    /* The caller provides time to the port. */
    (void)stamp_us;
}
int main(void)
{
    BF446MotorBenchPort port;
    const BMotorBenchConfig cfg={
        .rs06_motor_id=1u,.rs06_host_id=0xfdu,.c620_motor_id=2u,
        .rs06_limit_nm=0.3f,.c620_limit_a=0.25f,.pulse_duration_ms=100u
    };
    assert(b_f446_motor_port_init(&port,&can1,&can2,&cfg));
    b_f446_motor_port_tick_2ms(&port, 0u);
    assert(test_can2.tx_count==1u && test_can2.header.StdId==0x200u);
    for (int i=0;i<8;i++) assert(test_can2.data[i]==0u);
    b_f446_motor_port_on_tx_done(&port,&can2,0u,100u);

    assert(b_f446_motor_port_pulse_rs06(&port,0.2f,1000u));
    b_f446_motor_port_tick_2ms(&port,2000u); /* RS call on 2nd tick */
    assert(test_can1.tx_count==1u && (test_can1.header.ExtId >>24)==3u);
    b_f446_motor_port_on_tx_done(&port,&can1,0u,2100u);
    b_f446_motor_port_on_tx_done(&port,&can2,0u,2100u);

    rx_rs_good(2200u);
    b_f446_motor_port_on_rx0(&port,&can1,2200u);
    b_f446_motor_port_tick_2ms(&port,3000u);
    b_f446_motor_port_on_tx_done(&port,&can2,0u,3100u);
    b_f446_motor_port_tick_2ms(&port,4000u);
    assert(test_can1.tx_count==2u && (test_can1.header.ExtId>>24)==1u);
    b_f446_motor_port_on_tx_done(&port,&can1,0u,4200u);
    b_f446_motor_port_on_tx_done(&port,&can2,0u,4200u);

    b_f446_motor_port_cancel(&port);
    b_f446_motor_port_tick_2ms(&port,5000u);
    b_f446_motor_port_on_tx_done(&port,&can2,0u,5100u);
    b_f446_motor_port_tick_2ms(&port,6000u);
    assert(test_can1.tx_count==3u && (test_can1.header.ExtId>>24)==4u);
    b_f446_motor_port_on_tx_done(&port,&can1,0u,6200u);
    b_f446_motor_port_on_tx_done(&port,&can2,0u,6200u);
    assert(port.bench.state == B_BENCH_IDLE);

    /* C620 fresh reply is required before pulse. */
    assert(!b_f446_motor_port_pulse_c620(&port,0.2f,8000u));
    test_can2.rx_header=(CAN_RxHeaderTypeDef){
        .IDE=CAN_ID_STD,.RTR=CAN_RTR_DATA,.DLC=8u,.StdId=0x202u
    };
    const uint8_t wheel_fb[8]={0,0,0,0,0,0,26,0};
    memcpy(test_can2.rx_data,wheel_fb,8);
    test_can2.rx_pending=true;
    b_f446_motor_port_on_rx0(&port,&can2,8000u);
    assert(b_f446_motor_port_pulse_c620(&port,0.2f,8001u));
    b_f446_motor_port_tick_2ms(&port,9000u);
    assert(test_can2.data[2]!=0u || test_can2.data[3]!=0u);
    b_f446_motor_port_on_tx_done(&port,&can2,0u,9100u);
    b_f446_motor_port_tick_2ms(&port,40000u); /* Feedback stale => zero + FAILED */
    for (int i=0;i<8;i++) assert(test_can2.data[i]==0u);
    assert(port.bench.state == B_BENCH_FAILED);
    puts("合格：F446 HALアダプタの送受信・送信完了・短時間パルス（ホスト模擬）");
    return 0;
}
