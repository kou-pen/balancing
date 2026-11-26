#include "balancing/device/Bno055.hpp"
#include "balancing/device/I2cPort.hpp"

namespace balancing {
namespace {
// Bosch BST-BNO055-DS000-18 pp.36,40-41,56,92。非融合モードのACC/GYRは別更新。
constexpr std::uint8_t drdy = 0x11u;
constexpr std::uint64_t configWaitUs = 19000u;
constexpr std::uint64_t runWaitUs = 7000u;
float signedWord(const std::array<std::uint8_t,18>& bytes, std::size_t i) noexcept
{
    const int raw = static_cast<int>(bytes[i]) | (static_cast<int>(bytes[i+1]) << 8);
    return static_cast<float>(raw >= 32768 ? raw - 65536 : raw);
}
}
Bno055::Bno055(I2cPort& port, Bno055Config config) noexcept
    : port_{port}, config_{config},
      state_{port.state() == I2cState::Idle && config.maxRequestWaitUs
             ? Bno055State::Initializing : Bno055State::Unverified}
{}
void Bno055::poll(std::uint64_t nowUs) noexcept
{
    if (state_ == Bno055State::Failed || state_ == Bno055State::Unverified) return;
    if (lastPoll_ && nowUs < *lastPoll_) {
        state_ = Bno055State::Failed; sample_.reset(); return;
    }
    lastPoll_ = nowUs;
    const auto bus = port_.poll(nowUs);
    if (bus == I2cState::Error || bus == I2cState::TimedOut || bus == I2cState::Unverified) {
        state_ = Bno055State::Failed; sample_.reset(); return;
    }
    if (bus == I2cState::Pending) return;
    if (bus == I2cState::Complete) {
        if (const auto result = port_.takeCompletion()) complete(*result);
        requestSince_.reset();
        return; // 1回のpollで転送完了と次要求を連続処理しない
    }
    if (step_ == Step::WaitConfig || step_ == Step::WaitRun) {
        const auto wait = step_ == Step::WaitConfig ? configWaitUs : runWaitUs;
        if (nowUs - waitSince_ >= wait) {
            step_ = step_ == Step::WaitConfig ? Step::Units : Step::CheckMode;
        }
        return;
    }
    if (!requestSince_) requestSince_ = nowUs;
    if (nowUs - *requestSince_ >= config_.maxRequestWaitUs) {
        state_ = Bno055State::Failed; sample_.reset(); return;
    }
    I2cState result = I2cState::Error;
    switch (step_) {
    case Step::Page0: case Step::Page0Again: result = port_.requestWrite(0x07,0,nowUs); break;
    case Step::Identity: result = port_.requestRead(0x00,6,nowUs); break;
    case Step::ConfigMode: result = port_.requestWrite(0x3d,0,nowUs); break;
    case Step::Units: result = port_.requestWrite(0x3b,0x82,nowUs); break;
    case Step::Map: result = port_.requestRead(0x41,2,nowUs); break;
    case Step::MapCheck: result = port_.requestRead(0x3b,1,nowUs); break;
    case Step::Page1: result = port_.requestWrite(0x07,1,nowUs); break;
    case Step::Enable: result = port_.requestWrite(0x10,drdy,nowUs); break;
    case Step::EnableCheck: result = port_.requestRead(0x10,1,nowUs); break;
    case Step::Run: result = port_.requestWrite(0x3d,5,nowUs); break;
    case Step::CheckMode: result = port_.requestRead(0x3d,1,nowUs); break;
    case Step::Flush: case Step::Status: result = port_.requestRead(0x37,1,nowUs); break;
    case Step::Data: result = port_.requestRead(0x08,18,nowUs); break;
    default: break;
    }
    if (result != I2cState::Pending && result != I2cState::Busy) {
        state_ = Bno055State::Failed; sample_.reset();
    }
}
void Bno055::complete(const I2cCompletion& r) noexcept
{
    auto fail = [&] { state_ = Bno055State::Failed; sample_.reset(); };
    switch (step_) {
    case Step::Page0: step_=Step::Identity; break;
    case Step::Identity:
        // DRDYの利用にはSW revision 03.14以上が必要。版不明では代用しない。
        if (r.bytes[0]!=0xa0 || ((unsigned(r.bytes[5])<<8)|r.bytes[4]) < 0x0314u) { fail(); break; }
        step_=Step::ConfigMode; break;
    case Step::ConfigMode: waitSince_=r.completedAtUs; step_=Step::WaitConfig; break;
    case Step::Units: step_=Step::Map; break;
    case Step::Map:
        // センサ内部の軸変更を勝手に取り込まない。取付変換は姿勢フィルタが所有。
        if (r.bytes[0]!=0x24 || r.bytes[1]!=0) { fail(); break; }
        step_=Step::MapCheck; break;
    case Step::MapCheck:
        if ((r.bytes[0]&0x03u)!=0x02u) { fail(); break; }
        step_=Step::Page1; break;
    case Step::Page1: step_=Step::Enable; break;
    case Step::Enable: step_=Step::EnableCheck; break;
    case Step::EnableCheck:
        if ((r.bytes[0]&drdy)!=drdy) { fail(); break; }
        step_=Step::Page0Again; break;
    case Step::Page0Again: step_=Step::Run; break;
    case Step::Run: waitSince_=r.completedAtUs; step_=Step::WaitRun; break;
    case Step::CheckMode:
        if ((r.bytes[0]&0x0fu)!=5) { fail(); break; }
        state_=Bno055State::Observing; step_=Step::Flush; break;
    case Step::Flush:
        // バースト読み出し中のDRDYが次の同一データを再認定しないように捨てる。
        // read-clear時刻は転送区間内にあるため要求時刻を保守的な下限とする。
        lowerBound_=r.startedAtUs; ready_=0; step_=Step::Status; break;
    case Step::Status:
        ready_ |= r.bytes[0]&drdy;
        if (ready_==drdy) step_=Step::Data;
        else if (config_.maxEvidenceWindowUs &&
                 r.completedAtUs-lowerBound_ >= config_.maxEvidenceWindowUs) step_=Step::Flush;
        break;
    case Step::Data: {
        TimedImuSample next{};
        for (std::size_t i=0;i<3;++i) {
            next.value.accel_m_s2[i]=signedWord(r.bytes,2*i)/100.0f;
            next.value.gyro_rad_s[i]=signedWord(r.bytes,12+2*i)/900.0f;
        }
        next.earliestUs=lowerBound_;
        next.completedAtUs=r.completedAtUs;
        // ACC/GYRの同時刻性は主張しない。窓の妥当性を実測確認した場合だけ利用可。
        next.newMeasurementProven=config_.timingVerified && config_.maxEvidenceWindowUs &&
            r.completedAtUs-lowerBound_ < config_.maxEvidenceWindowUs;
        sample_=next; step_=Step::Flush; break;
    }
    default: fail(); break;
    }
}
std::optional<TimedImuSample> Bno055::takeSample() noexcept
{
    auto result=sample_; sample_.reset(); return result;
}
}
