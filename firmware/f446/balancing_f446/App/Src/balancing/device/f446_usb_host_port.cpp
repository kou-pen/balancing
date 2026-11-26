#include "balancing/device/F446UsbHostPort.hpp"
#include "balancing/app/ApplicationObservation.hpp"
#include "balancing/device/MotorObservation.hpp"
#include "usbh_core.h"
#include "usbh_pipes.h"

#include <array>
#include <limits>

namespace {
using namespace balancing;
constexpr std::uint8_t kNoPipe=0xffu;
constexpr std::uint32_t kPortChangeMask=USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET |
    USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG;
enum class DrainPhase : std::uint8_t { Idle,Halt,TxFlush,RxFlush,Blocked };
enum class ResetPhase : std::uint8_t { Idle,Asserted,Recovery };

// この実装だけがSTM32/HCD/USBHを知る。Applicationは固定長の完了通知だけを受ける。
struct F446Host {
    F446UsbHostBoardConfig board{};
    UsbHostConfig config{};
    USBH_HandleTypeDef host{};
    UsbHostInterface selected{};
    std::uint8_t pipe{kNoPipe};
    alignas(4) std::array<std::uint8_t,64> buffer{};
    bool attempted{},attached{},ready{},quarantined{},llFailed{},waitUnplug{};
    volatile bool running{},announced{};
    std::uint64_t attachment{},attachedAt{},drainStarted{},transferStarted{};
    DrainPhase drain{DrainPhase::Idle};
    ResetPhase reset{ResetPhase::Idle};
    std::uint32_t resetAtMs{};
    // ISRはフラグ・時刻・転送結果を保存するだけ。mainとのコピーは短い排他区間。
    volatile bool detachedIrq{},activeTransfer{},terminalPending{},terminalOverflow{};
    UsbTransferToken token{};
    UsbHostEvent terminal{};

    USB_OTG_GlobalTypeDef* regs() const noexcept { return board.hcd->Instance; }
    volatile std::uint32_t& port() const noexcept {
        return *reinterpret_cast<volatile std::uint32_t*>(
            reinterpret_cast<std::uintptr_t>(regs())+USB_OTG_HOST_PORT_BASE);
    }
    USB_OTG_HostChannelTypeDef& channel(std::uint8_t ch) const noexcept {
        return *reinterpret_cast<USB_OTG_HostChannelTypeDef*>(
            reinterpret_cast<std::uintptr_t>(regs())+USB_OTG_HOST_CHANNEL_BASE+
            static_cast<std::uintptr_t>(ch)*USB_OTG_HOST_CHANNEL_SIZE);
    }
    bool pipeValid(std::uint8_t ch) const noexcept
    { return board.hcd && ch<board.hcd->Init.Host_channels && ch<16u; }
    bool physicallyPresent() const noexcept { return (port()&USB_OTG_HPRT_PCSTS)!=0u; }
    bool portEnabled() const noexcept { return (port()&USB_OTG_HPRT_PENA)!=0u; }

    void releaseReset() noexcept {
        if (reset==ResetPhase::Asserted) port()=port()&~(kPortChangeMask|USB_OTG_HPRT_PRST);
        reset=ResetPhase::Idle;
    }

    void beginDrain(std::uint64_t now,bool requireUnplug=false) noexcept {
        // HCD_Stop/USB_StopHostは待機ループを含むためここでは呼ばない。
        // IRQを止め、チャネル停止要求だけ発行し、完了は後続pollで確認する。
        HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
        __HAL_HCD_DISABLE(board.hcd);
        releaseReset();
        activeTransfer=false; terminalPending=false; terminalOverflow=false;
        ready=false; announced=false; attached=false; quarantined=false;
        waitUnplug=requireUnplug;
        for (std::uint8_t i=0;i<board.hcd->Init.Host_channels;++i) {
            auto& hc=channel(i);
            hc.HCINTMSK=0u;
            if ((hc.HCCHAR&USB_OTG_HCCHAR_CHENA)!=0u)
                hc.HCCHAR |= USB_OTG_HCCHAR_CHDIS | USB_OTG_HCCHAR_CHENA;
        }
        drainStarted=now; drain=DrainPhase::Halt;
    }

    void advanceDrain(std::uint64_t now) noexcept;
    UsbHostEvent poll(std::uint64_t now) noexcept;
    bool submit(UsbTransferToken newToken,std::uint8_t bytes) noexcept;
    void captureTerminal(std::uint8_t ch,HCD_URBStateTypeDef state) noexcept;
};
F446Host runtime{};

USBH_StatusTypeDef status(HAL_StatusTypeDef s) noexcept
{ return s==HAL_OK?USBH_OK:(s==HAL_BUSY?USBH_BUSY:USBH_FAIL); }

USBH_StatusTypeDef classInit(USBH_HandleTypeDef* h)
{
    // 複合デバイスの先頭interfaceを決め打ちしない。生descriptor全体を検査。
    const auto match=findDualSenseInterface(runtime.config,h->device.DevDesc.idVendor,
        h->device.DevDesc.idProduct,h->device.speed==USBH_SPEED_FULL,
        h->device.CfgDesc_Raw,h->device.CfgDesc.wTotalLength);
    if (!match) return USBH_FAIL;
    const auto index=USBH_FindInterfaceIndex(h,match->number,0u);
    if (index==kNoPipe || USBH_SelectInterface(h,index)!=USBH_OK) return USBH_FAIL;
    const auto p=USBH_AllocPipe(h,match->endpointAddress);
    if (!runtime.pipeValid(p)) return USBH_FAIL;
    if (USBH_OpenPipe(h,p,match->endpointAddress,h->device.address,
                      h->device.speed,USBH_EP_INTERRUPT,match->packetBytes)!=USBH_OK) {
        (void)USBH_FreePipe(h,p);
        return USBH_FAIL;
    }
    runtime.pipe=p; runtime.selected=*match;
    USBH_LL_SetToggle(h,p,0u);
    return USBH_OK;
}
USBH_StatusTypeDef classDeinit(USBH_HandleTypeDef* h)
{
    if (runtime.pipeValid(runtime.pipe)) (void)USBH_FreePipe(h,runtime.pipe);
    runtime.pipe=kNoPipe; runtime.ready=false;
    return USBH_OK;
}
USBH_StatusTypeDef classRequests(USBH_HandleTypeDef*)
{
    // 非BootのInterrupt IN。出力報告・Boot protocol変更は送らない。
    runtime.ready=true;
    return USBH_OK;
}
USBH_StatusTypeDef classIdle(USBH_HandleTypeDef*) { return USBH_OK; }
USBH_ClassTypeDef gamepadClass{"DualSense",3u,classInit,classDeinit,classRequests,
                              classIdle,classIdle,nullptr};
void userEvent(USBH_HandleTypeDef*,std::uint8_t) {}
bool initCore() noexcept
{
    runtime.host={};
    runtime.pipe=kNoPipe; runtime.ready=false; runtime.announced=false; runtime.llFailed=false;
    return USBH_Init(&runtime.host,userEvent,0u)==USBH_OK &&
           USBH_RegisterClass(&runtime.host,&gamepadClass)==USBH_OK;
}

void F446Host::advanceDrain(std::uint64_t now) noexcept
{
    if (drain==DrainPhase::Blocked || drain==DrainPhase::Idle) return;
    if (now-drainStarted>=board.recoveryTimeoutUs) {
        // 終端が確認できないハードを再利用しない。再起動/実機診断が必要。
        drain=DrainPhase::Blocked;
        return;
    }
    switch (drain) {
    case DrainPhase::Halt:
        for (std::uint8_t i=0;i<board.hcd->Init.Host_channels;++i)
            if ((channel(i).HCCHAR&USB_OTG_HCCHAR_CHENA)!=0u) return;
        if ((regs()->GRSTCTL&USB_OTG_GRSTCTL_AHBIDL)==0u) return;
        regs()->GRSTCTL=USB_OTG_GRSTCTL_TXFFLSH | (0x10u<<6u);
        drain=DrainPhase::TxFlush;
        break;
    case DrainPhase::TxFlush:
        if ((regs()->GRSTCTL&USB_OTG_GRSTCTL_TXFFLSH)!=0u ||
            (regs()->GRSTCTL&USB_OTG_GRSTCTL_AHBIDL)==0u) return;
        regs()->GRSTCTL=USB_OTG_GRSTCTL_RXFFLSH;
        drain=DrainPhase::RxFlush;
        break;
    case DrainPhase::RxFlush:
        if ((regs()->GRSTCTL&USB_OTG_GRSTCTL_RXFFLSH)!=0u) return;
        for (std::uint8_t i=0;i<board.hcd->Init.Host_channels;++i) {
            channel(i).HCINT=0xffffffffu;
            board.hcd->hc[i]={};
        }
        // 旧転送完了とバッファを隔離した後にのみ列挙状態を初期化。
        detachedIrq=false;
        if (!initCore() || USBH_LL_Start(&host)!=USBH_OK) {
            drain=DrainPhase::Blocked; return;
        }
        HAL_NVIC_ClearPendingIRQ(OTG_HS_IRQn);
        drain=DrainPhase::Idle;
        HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
        break;
    default: break;
    }
}

UsbHostEvent F446Host::poll(std::uint64_t now) noexcept
{
    if (!running) return {};
    if (drain!=DrainPhase::Idle) { advanceDrain(now); return {}; }
    const auto irqMask=__get_PRIMASK();
    __disable_irq();
    const bool lost=detachedIrq;
    detachedIrq=false;
    // now採取直後のIRQは未来扱いで捨てず、次のpollで回収する。
    const bool done=terminalPending && terminal.atUs<=now;
    const bool overflow=terminalOverflow;
    auto event=terminal;
    if (done) { terminalPending=false; terminalOverflow=false; }
    __set_PRIMASK(irqMask);
    if (attached && (lost || !physicallyPresent() || (announced && !portEnabled()))) {
        const auto old=attachment;
        beginDrain(now);
        return {UsbHostEventKind::Detached,old,now};
    }
    if (waitUnplug) {
        if (physicallyPresent()) return {};
        waitUnplug=false;
    }
    if (!attached) {
        if (!physicallyPresent()) return {};
        if (attachment==std::numeric_limits<std::uint64_t>::max()) return {};
        ++attachment; attached=true; attachedAt=now;
        host.device.is_connected=1u;
    }
    if (done && announced) {
        // ISR終了後の固定bufferをコピー。次のsubmitまでは再使用しない。
        if (overflow || now-transferStarted>=config.transferTimeoutUs)
            event.kind=UsbHostEventKind::Error;
        if (event.kind==UsbHostEventKind::Done && event.length<=buffer.size())
            event.data=buffer;
        if (event.kind==UsbHostEventKind::Stall || event.kind==UsbHostEventKind::Error)
            beginDrain(now,true);
        return event;
    }
    if (activeTransfer && now-transferStarted>=config.transferTimeoutUs) {
        beginDrain(now,true);
        return {UsbHostEventKind::Error,attachment,now};
    }
    if (quarantined || llFailed || host.gState==HOST_ABORT_STATE ||
        (!announced && now-attachedAt>=board.enumerationTimeoutUs)) {
        // 失敗した列挙・pipeをその接続中に再開しない。
        beginDrain(now,true);
        return {UsbHostEventKind::Error,attachment,now};
    }
    host.device.PortEnabled=portEnabled()?1u:0u;
    host.Timer=HAL_GetTick();
    (void)USBH_Process(&host); // 待機は全て状態遷移。次のpollまで戻る。
    if (ready && !announced && host.gState==HOST_CLASS) {
        announced=true;
        return {UsbHostEventKind::Ready,attachment,now,selected};
    }
    return {};
}

bool F446Host::submit(UsbTransferToken newToken,std::uint8_t bytes) noexcept
{
    if (!announced || quarantined || !pipeValid(pipe) || bytes!=config.packetBytes ||
        bytes>buffer.size() || !physicallyPresent() || !portEnabled()) return false;
    const auto now=observationClockUs();
    if (!now) return false;
    const auto irqMask=__get_PRIMASK();
    __disable_irq();
    if (activeTransfer || terminalPending || (channel(pipe).HCCHAR&USB_OTG_HCCHAR_CHENA)!=0u) {
        __set_PRIMASK(irqMask); return false;
    }
    token=newToken; transferStarted=*now; activeTransfer=true;
    __set_PRIMASK(irqMask);
    // HALは一件の固定バッファを借用。受付と完了を混同しない。
    const bool accepted=USBH_LL_SubmitURB(&host,pipe,1u,USBH_EP_INTERRUPT,USBH_PID_DATA,
                                          buffer.data(),bytes,0u)==USBH_OK;
    if (!accepted) {
        const auto mask=__get_PRIMASK(); __disable_irq(); activeTransfer=false; __set_PRIMASK(mask);
        quarantined=true;
    }
    return accepted;
}

void F446Host::captureTerminal(std::uint8_t ch,HCD_URBStateTypeDef urb) noexcept
{
    if (!running || ch!=pipe || !activeTransfer) return;
    UsbHostEventKind kind{};
    switch (urb) {
    case URB_DONE: kind=UsbHostEventKind::Done; break;
    case URB_NOTREADY: kind=UsbHostEventKind::Nak; break;
    case URB_STALL: kind=UsbHostEventKind::Stall; break;
    case URB_ERROR: kind=UsbHostEventKind::Error; break;
    default: return;
    }
    const auto at=observationClockUs();
    // HAL内部再試行でチャネルがまだ有効なら新しいURBに使い回さない。
    if (!at || (channel(ch).HCCHAR&USB_OTG_HCCHAR_CHENA)!=0u)
        kind=UsbHostEventKind::Error;
    if (terminalPending) terminalOverflow=true;
    else terminal={kind,attachment,at.value_or(0u),{},token,
                   HAL_HCD_HC_GetXferCount(board.hcd,ch)};
    terminalPending=true;
    activeTransfer=false;
}

bool backendStart(void*,const UsbHostConfig& config) noexcept
{
    if (runtime.attempted) return false;
    runtime.attempted=true;
    runtime.config=config;
    HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
    HAL_NVIC_SetPriority(OTG_HS_IRQn,runtime.board.irqPriority,0u);
    if (!initCore() || USBH_Start(&runtime.host)!=USBH_OK) {
        (void)runtime.board.setVbus(runtime.board.vbusContext,false);
        return false;
    }
    runtime.running=true;
    HAL_NVIC_ClearPendingIRQ(OTG_HS_IRQn);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
    return true;
}
}

bool balancing::startF446UsbHost(const F446UsbHostBoardConfig& board,const UsbHostConfig& config) noexcept
{
    if (runtime.attempted || !validUsbHostConfig(config) || !board.verified ||
        !board.hcd || board.hcd->Instance!=USB_OTG_HS || !board.setVbus ||
        !board.enumerationTimeoutUs || !board.recoveryTimeoutUs ||
        board.irqPriority>= (1u<<__NVIC_PRIO_BITS) ||
        board.hcd->Init.Host_channels<3u || board.hcd->Init.Host_channels>16u ||
        board.hcd->Init.dma_enable!=DISABLE ||
        board.hcd->Init.phy_itface!=USB_OTG_EMBEDDED_PHY ||
        HAL_HCD_GetState(board.hcd)!=HAL_HCD_STATE_READY) return false;
    // USB IRQはTIM7より低いプリエンプト優先度でのみ有効化する。
    std::uint32_t timerPriority{},timerSub{};
    NVIC_DecodePriority(NVIC_GetPriority(TIM7_IRQn),NVIC_GetPriorityGrouping(),
                        &timerPriority,&timerSub);
    const auto encoded=NVIC_EncodePriority(NVIC_GetPriorityGrouping(),board.irqPriority,0u);
    std::uint32_t usbPriority{},usbSub{};
    NVIC_DecodePriority(encoded,NVIC_GetPriorityGrouping(),&usbPriority,&usbSub);
    if (usbPriority!=board.irqPriority || usbPriority<=timerPriority) return false;
    runtime.board=board;
    const UsbHostBackend backend{&runtime,backendStart,
        [](void* c,std::uint64_t now) noexcept { return static_cast<F446Host*>(c)->poll(now); },
        [](void* c,UsbTransferToken token,std::uint8_t bytes) noexcept {
            return static_cast<F446Host*>(c)->submit(token,bytes);
        }};
    return startApplicationUsbHost(config,backend);
}

// ST USBH CoreのLL境界。HALのブロッキングreset/stopは実行しない。
extern "C" {
USBH_StatusTypeDef USBH_LL_Init(USBH_HandleTypeDef* h)
{ h->pData=runtime.board.hcd; return runtime.board.hcd?USBH_OK:USBH_FAIL; }
USBH_StatusTypeDef USBH_LL_DeInit(USBH_HandleTypeDef*) { return USBH_FAIL; }
USBH_StatusTypeDef USBH_LL_Start(USBH_HandleTypeDef*)
{ return status(HAL_HCD_Start(runtime.board.hcd)); }
USBH_StatusTypeDef USBH_LL_Stop(USBH_HandleTypeDef*) { return USBH_FAIL; }
USBH_SpeedTypeDef USBH_LL_GetSpeed(USBH_HandleTypeDef*)
{ return static_cast<USBH_SpeedTypeDef>(HAL_HCD_GetCurrentSpeed(runtime.board.hcd)); }
std::uint32_t USBH_LL_Millis(void) { return HAL_GetTick(); }
USBH_StatusTypeDef USBH_LL_ResetPort(USBH_HandleTypeDef*)
{
    if (!runtime.physicallyPresent()) { runtime.releaseReset(); return USBH_FAIL; }
    const auto now=HAL_GetTick();
    switch (runtime.reset) {
    case ResetPhase::Idle:
        runtime.port()=(runtime.port()&~kPortChangeMask)|USB_OTG_HPRT_PRST;
        runtime.resetAtMs=now; runtime.reset=ResetPhase::Asserted; return USBH_BUSY;
    case ResetPhase::Asserted:
        if (now-runtime.resetAtMs<100u) return USBH_BUSY;
        runtime.releaseReset(); runtime.resetAtMs=now; runtime.reset=ResetPhase::Recovery;
        return USBH_BUSY;
    case ResetPhase::Recovery:
        if (now-runtime.resetAtMs<10u) return USBH_BUSY;
        runtime.reset=ResetPhase::Idle; return USBH_OK;
    }
    return USBH_FAIL;
}
std::uint32_t USBH_LL_GetLastXferSize(USBH_HandleTypeDef*,std::uint8_t pipe)
{ return runtime.pipeValid(pipe)?HAL_HCD_HC_GetXferCount(runtime.board.hcd,pipe):0u; }
USBH_StatusTypeDef USBH_LL_DriverVBUS(USBH_HandleTypeDef*,std::uint8_t state)
{ return runtime.board.setVbus(runtime.board.vbusContext,state!=0u)?USBH_OK:USBH_FAIL; }
USBH_StatusTypeDef USBH_LL_OpenPipe(USBH_HandleTypeDef*,std::uint8_t pipe,std::uint8_t epnum,
    std::uint8_t address,std::uint8_t speed,std::uint8_t type,std::uint16_t mps)
{
    if (!runtime.pipeValid(pipe) || (runtime.channel(pipe).HCCHAR&USB_OTG_HCCHAR_CHENA)!=0u ||
        !mps || mps>64u) { runtime.llFailed=true; return USBH_FAIL; }
    const auto result=status(HAL_HCD_HC_Init(runtime.board.hcd,pipe,epnum,address,speed,type,mps));
    if (result!=USBH_OK) runtime.llFailed=true;
    return result;
}
USBH_StatusTypeDef USBH_LL_ActivatePipe(USBH_HandleTypeDef*,std::uint8_t) { return USBH_FAIL; }
USBH_StatusTypeDef USBH_LL_ClosePipe(USBH_HandleTypeDef*,std::uint8_t pipe)
{
    if (!runtime.pipeValid(pipe)) return USBH_FAIL;
    auto& hc=runtime.channel(pipe);
    if ((hc.HCCHAR&USB_OTG_HCCHAR_CHENA)!=0u) {
        hc.HCCHAR |= USB_OTG_HCCHAR_CHDIS | USB_OTG_HCCHAR_CHENA;
        return USBH_BUSY;
    }
    hc.HCINTMSK=0u;
    return USBH_OK;
}
USBH_StatusTypeDef USBH_LL_SubmitURB(USBH_HandleTypeDef*,std::uint8_t pipe,std::uint8_t direction,
    std::uint8_t type,std::uint8_t token,std::uint8_t* data,std::uint16_t length,std::uint8_t ping)
{
    if (!runtime.pipeValid(pipe) || (runtime.channel(pipe).HCCHAR&USB_OTG_HCCHAR_CHENA)!=0u ||
        (length && !data) || length>512u) { runtime.llFailed=true; return USBH_FAIL; }
    const auto result=status(HAL_HCD_HC_SubmitRequest(runtime.board.hcd,pipe,direction,type,token,
                                                     data,length,ping));
    if (result!=USBH_OK) runtime.llFailed=true;
    return result;
}
USBH_URBStateTypeDef USBH_LL_GetURBState(USBH_HandleTypeDef*,std::uint8_t pipe)
{
    if (!runtime.pipeValid(pipe)) return USBH_URB_ERROR;
    switch (HAL_HCD_HC_GetURBState(runtime.board.hcd,pipe)) {
    case URB_DONE: return USBH_URB_DONE;
    case URB_NOTREADY: return USBH_URB_NOTREADY;
    case URB_NYET: return USBH_URB_NYET;
    case URB_ERROR: return USBH_URB_ERROR;
    case URB_STALL: return USBH_URB_STALL;
    default: return USBH_URB_IDLE;
    }
}
USBH_StatusTypeDef USBH_LL_SetToggle(USBH_HandleTypeDef*,std::uint8_t pipe,std::uint8_t toggle)
{
    if (!runtime.pipeValid(pipe) || toggle>1u) return USBH_FAIL;
    auto& hc=runtime.board.hcd->hc[pipe];
    if (hc.ep_is_in) hc.toggle_in=toggle; else hc.toggle_out=toggle;
    return USBH_OK;
}
std::uint8_t USBH_LL_GetToggle(USBH_HandleTypeDef*,std::uint8_t pipe)
{
    if (!runtime.pipeValid(pipe)) return 0u;
    const auto& hc=runtime.board.hcd->hc[pipe];
    return hc.ep_is_in?hc.toggle_in:hc.toggle_out;
}
void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef* h)
{ if (runtime.running && h==runtime.board.hcd) runtime.detachedIrq=true; }
void HAL_HCD_Connect_Callback(HCD_HandleTypeDef*) {}
void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef*) {}
void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef* h)
{
    // リセット中のPortDisabledは正常。列挙済みの喪失だけラッチする。
    if (runtime.running && h==runtime.board.hcd && runtime.announced) runtime.detachedIrq=true;
}
void HAL_HCD_SOF_Callback(HCD_HandleTypeDef*) {}
void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef* h,std::uint8_t ch,HCD_URBStateTypeDef urb)
{ if (runtime.running && h==runtime.board.hcd) runtime.captureTerminal(ch,urb); }
void OTG_HS_IRQHandler(void)
{ if (runtime.running) HAL_HCD_IRQHandler(runtime.board.hcd); }
}
