#include "balancing_f446_imu_bridge.h"
#include "balancing/device/ImuObservation.hpp"
#include "balancing/device/MotorObservation.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace balancing;
extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef*);
extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef*);
extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef*);
namespace {
I2C_TypeDef registers{};
std::uint32_t primask{};
std::uint64_t clockUs{};
struct Transfer {
    I2C_HandleTypeDef* handle{};
    std::uint16_t reg{},size{};
    std::uint8_t* buffer{};
    bool write{};
} pending;
unsigned calls{}, irqs{};
bool failStart{};
std::array<std::array<std::uint8_t,128>,2> regs{};
unsigned page{};
std::uint64_t enteredConfig{}, enteredRun{};
void setup()
{
    regs={};
    regs[0][0]=0xa0;
    regs[0][4]=0x14;
    regs[0][5]=3;
    regs[0][0x41]=0x24;
    page=0;
    pending={};
    calls=0;
    registers.SR2=0;
    failStart=false;
    enteredConfig=enteredRun=0;
}
void word(unsigned reg,int v)
{
    const auto value=static_cast<std::uint16_t>(v);
    regs[0][reg]=static_cast<std::uint8_t>(value);
    regs[0][reg+1]=static_cast<std::uint8_t>(value>>8);
}
void complete(I2cPort* port,bool error=false)
{
    assert(pending.handle);
    const auto req=pending; pending={};
    if (!error) {
        if (req.write) {
            if (req.reg==7) page=req.buffer[0];
            else {
                regs[page][req.reg]=req.buffer[0];
                if (page==0 && req.reg==0x3d) {
                    if (req.buffer[0]==0) enteredConfig=clockUs;
                    else enteredRun=clockUs;
                }
                if (page==0 && req.reg==0x3b) assert(clockUs-enteredConfig>=19000);
            }
        } else {
            if (page==0 && req.reg==0x37) assert(clockUs-enteredRun>=7000);
            for (unsigned i=0;i<req.size;++i) req.buffer[i]=regs[page][req.reg+i];
            if (page==0 && req.reg==0x37) regs[0][0x37]=0;
        }
    }
    req.handle->State=HAL_I2C_STATE_READY;
    if (port) port->onCompletion(req.handle,req.write,error);
    else if (error) HAL_I2C_ErrorCallback(req.handle);
    else if (req.write) HAL_I2C_MemTxCpltCallback(req.handle);
    else HAL_I2C_MemRxCpltCallback(req.handle);
}
HAL_StatusTypeDef begin(I2C_HandleTypeDef* h,std::uint16_t address,std::uint16_t reg,
    std::uint16_t mem,std::uint8_t* bytes,std::uint16_t size,bool write)
{
    assert(address==0x50 && mem==1 && !pending.handle);
    ++calls;
    if (failStart) return HAL_ERROR;
    pending={h,reg,size,bytes,write};
    h->State=HAL_I2C_STATE_BUSY;
    return HAL_OK;
}
BAttitudeFilterConfig filterConfig()
{
    BAttitudeFilterConfig config{};
    config.mount={{0,1,2},{1,1,1},true};
    config.bias_samples_required=2;
    config.gravity_min_m_s2=9;
    config.gravity_max_m_s2=11;
    config.gyro_alpha=0.9f;
    config.max_sample_dt_s=0.05f;
    config.verified=true;
    return config;
}
void near(float x,float y) { assert(std::fabs(x-y)<1e-5f); }
void testTransport()
{
    setup();
    I2C_HandleTypeDef h{I2C1,HAL_I2C_STATE_READY};
    I2cPort port{h,{5000,true}};
    registers.SR2=I2C_SR2_BUSY;
    assert(port.requestRead(0,1,0)==I2cState::Busy && calls==0);
    registers.SR2=0;
    assert(port.requestRead(0,1,0)==I2cState::Pending);
    assert(port.requestWrite(3,1,1)==I2cState::Pending && calls==1);
    I2C_HandleTypeDef other{I2C1,HAL_I2C_STATE_READY};
    port.onCompletion(&other,false,false);
    assert(port.poll(2)==I2cState::Pending);
    complete(&port);
    assert(port.poll(3)==I2cState::Complete);
    assert(port.requestRead(0,1,3)==I2cState::Complete);
    assert(port.takeCompletion()->bytes[0]==0xa0);
    assert(port.requestRead(0,1,4)==I2cState::Pending);
    assert(port.poll(5004)==I2cState::TimedOut);
    complete(&port); // 期限後の通知を次の転送完了に流用しない。
    assert(port.poll(5005)==I2cState::TimedOut && !port.takeCompletion());
    assert(port.requestRead(0,1,5006)==I2cState::TimedOut && calls==2);
    I2cPort nack{h,{5000,true}};
    assert(nack.requestRead(0,1,6000)==I2cState::Pending);
    complete(&nack,true);
    assert(nack.poll(6001)==I2cState::Error);
    assert(!nack.takeCompletion());
    I2cPort disabled{h,{}};
    assert(disabled.requestRead(0,1,0)==I2cState::Unverified);
}
void testAttitudeSourceFreshness()
{
    ImuAttitudeSource source{filterConfig()};
    assert(!source.attitude(0).valid);
    TimedImuSample sample{{{0,0,9.81f},{0,0.125f,0}},1000,1001,true};
    auto unproven=sample;unproven.newMeasurementProven=false;
    assert(!source.accept(unproven,true,1001));
    assert(!source.accept(sample,true,999)); // 未来完了は受理しない
    assert(source.accept(sample,true,1001));
    assert(!source.accept(sample,true,1002));
    sample.earliestUs=2000;sample.completedAtUs=2001;
    assert(source.accept(sample,true,2001));
    assert(!source.attitude(2001).valid);
    sample.earliestUs=3000;sample.completedAtUs=3001;
    sample.value.gyro_rad_s[1]=0.2f;
    assert(source.accept(sample,false,3001));
    const auto frozen=source.attitude(3001);
    assert(frozen.valid && frozen.fresh && frozen.receivedAtUs==3000);
    near(frozen.pitchRateRadS,0.075f);
    // 何度コピーしても元の測定時刻を進めない。現行期限は50ms未満。
    assert(source.attitude(52999).receivedAtUs==3000 && source.attitude(52999).fresh);
    assert(!source.attitude(53000).fresh);
    assert(!source.attitude(2999).valid);
    assert(!source.accept(sample,false,4000));
    sample.earliestUs=4000;sample.completedAtUs=4001;
    sample.value.gyro_rad_s[1]=-0.075f;
    assert(source.accept(sample,false,4001));
    near(source.attitude(4001).pitchRateRadS,-0.2f);
    near(frozen.pitchRateRadS,0.075f);
    assert(frozen.receivedAtUs==3000);
    sample.earliestUs=5000;sample.completedAtUs=5001;
    sample.value.gyro_rad_s[2]=std::numeric_limits<float>::quiet_NaN();
    assert(!source.accept(sample,false,5001));
    // 現行の姿勢境界は異常測定でラッチ無効。古い有効値で制御を続けない。
    assert(!source.attitude(5001).valid);
    assert(source.attitude(5001).receivedAtUs==4000);
}

void testBno()
{
    setup();
    I2C_HandleTypeDef h{I2C1,HAL_I2C_STATE_READY};
    I2cPort port{h,{5000,true}};
    // 20msの証拠窓は人工試験条件であり、実機値ではない。
    Bno055 sensor{port,{20000,5000,true}};
    clockUs=0;
    auto advance=[&]{
        clockUs+=100;
        sensor.poll(clockUs);
        if (pending.handle) complete(&port);
    };
    for(unsigned i=0;i<500 && sensor.state()!=Bno055State::Observing;++i) advance();
    assert(sensor.state()==Bno055State::Observing);
    assert(regs[1][0x10]==0x11 && regs[0][0x3d]==5);
    // Flushで既存DRDYを捨て、片側だけではバーストを開始しない。
    regs[0][0x37]=0x11;
    for(unsigned i=0;i<4;++i) advance();
    assert(!sensor.takeSample());
    regs[0][0x37]=1;
    for(unsigned i=0;i<4;++i) advance();
    assert(!sensor.takeSample());
    word(0x08,-100);word(0x0a,0);word(0x0c,980);
    word(0x14,-900);word(0x16,450);word(0x18,0);
    regs[0][0x37]=0x10;
    std::optional<TimedImuSample> sample;
    for(unsigned i=0;i<20 && !sample;++i) { advance();sample=sensor.takeSample(); }
    assert(sample && sample->newMeasurementProven && sample->earliestUs<sample->completedAtUs);
    near(sample->value.accel_m_s2[0],-1); near(sample->value.accel_m_s2[2],9.8f);
    near(sample->value.gyro_rad_s[0],-1);near(sample->value.gyro_rad_s[1],0.5f);
    // バースト中に立った通知は次のflushで消費する。
    regs[0][0x37]=0x11;
    for(unsigned i=0;i<20;++i) advance();
    assert(!sensor.takeSample());
    ImuAttitudeSource source{filterConfig()};
    assert(!source.accept(*sample,false,clockUs));
    assert(source.accept(*sample,true,clockUs));
    assert(!source.accept(*sample,true,clockUs)); // 同じ校正測定の再利用禁止
    auto next=*sample; next.earliestUs+=1000;next.completedAtUs+=1000;
    assert(source.accept(next,true,clockUs+1000));
    next.earliestUs+=1000;next.completedAtUs+=1000;
    assert(source.accept(next,false,clockUs+2000));
    const auto attitude=source.attitude(clockUs+2000);
    assert(attitude.valid && attitude.fresh && attitude.receivedAtUs==next.earliestUs);
    near(attitude.rollRateRadS,0);
    assert(!source.attitude(next.earliestUs+50000).fresh);
    assert(!source.accept(next,false,clockUs+2000));
    source.invalidate();assert(!source.attitude(clockUs+2000).valid);
    // 不対応版は設定・測定を続行しない。
    setup();regs[0][4]=0x13;
    I2cPort oldPort{h,{5000,true}};
    Bno055 old{oldPort,{20000,5000,true}};
    for(unsigned i=0;i<20;++i) {
        clockUs+=100;old.poll(clockUs);if(pending.handle) complete(&oldPort);
    }
    assert(old.state()==Bno055State::Failed && !old.takeSample());
}
}
I2C_TypeDef* I2C1=&registers;
std::uint32_t __get_PRIMASK(){return primask;}
void __disable_irq(){primask=1;}
void __enable_irq(){primask=0;}
void __set_PRIMASK(std::uint32_t mask){primask=mask;}
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef* h,std::uint16_t a,std::uint16_t r,std::uint16_t m,std::uint8_t* b,std::uint16_t n)
{return begin(h,a,r,m,b,n,false);}
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef* h,std::uint16_t a,std::uint16_t r,std::uint16_t m,std::uint8_t* b,std::uint16_t n)
{return begin(h,a,r,m,b,n,true);}
void HAL_NVIC_SetPriority(int,int,int){}
void HAL_NVIC_EnableIRQ(int){++irqs;}
std::optional<std::uint64_t> balancing::observationClockUs() noexcept {return clockUs;}
int main()
{
    testTransport();
    testAttitudeSourceFreshness();
    testBno();
    setup();
    I2C_HandleTypeDef h{I2C1,HAL_I2C_STATE_READY};
    assert(!balancing_f446_imu_start(&h));
    balancing_f446_imu_poll();
    assert(calls==0 && irqs==0 && !imuAttitude(clockUs).valid);
    // 実行時ブリッジも構築し、実HAL callbackと同じ入口から完了させる。
    assert(startImuObservation(&h,{{5000,true},{20000,5000,true},filterConfig()}));
    assert(irqs==2);
    for (unsigned i=0;i<500;++i) {
        clockUs+=100;
        balancing_f446_imu_poll();
        if (pending.handle) complete(nullptr);
        if (regs[0][0x3d]==5) regs[0][0x37]=0x11;
    }
    assert(calls>20);
    assert(!imuAttitude(clockUs).valid); // 実機の独立静止証明なし
    std::puts("PASS: asynchronous I2C, BNO055 DRDY and attitude source");
}
