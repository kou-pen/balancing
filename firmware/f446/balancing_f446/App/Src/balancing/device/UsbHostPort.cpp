#include "balancing/device/UsbHostPort.hpp"
#include <limits>

namespace balancing {
bool UsbHostPort::start(UsbHostConfig config,UsbHostBackend backend) noexcept
{
    if (summary_.started || !validUsbHostConfig(config) || !backend.start ||
        !backend.poll || !backend.submit) return false;
    if (!backend.start(backend.context,config)) return false;
    config_=config; backend_=backend; summary_.started=true;
    return true;
}

void UsbHostPort::loseConnection(std::uint64_t atUs) noexcept
{
    // 古い転送の完了を別世代へ引き継がない。再開には新しいattachmentが必要。
    if (summary_.connected) {
        if (atUs<lastBoundary_) atUs=lastBoundary_;
        connection_=UsbHostConnection{generation_,atUs,false};
        lastBoundary_=atUs;
    }
    summary_.connected=false;
    summary_.inFlight=false;
    report_.reset();
    lastSubmit_.reset();
}

void UsbHostPort::handle(const UsbHostEvent& e,std::uint64_t nowUs) noexcept
{
    if (e.kind==UsbHostEventKind::None) return;
    if (!e.attachment || e.atUs>nowUs) { ++summary_.rejectedEvents; return; }
    if (e.kind==UsbHostEventKind::Ready) {
        if (e.attachment<=attachment_ || e.atUs<lastBoundary_) {
            ++summary_.rejectedEvents; return;
        }
        // 切断通知なしの世代変更では操作を無効にする。新規接続は次の列挙から。
        const bool wasConnected=summary_.connected;
        attachment_=e.attachment;
        if (wasConnected) {
            ++summary_.faults; loseConnection(e.atUs); return;
        }
        if (!matchesDualSenseInterface(config_,e.interface) ||
            generation_==std::numeric_limits<std::uint64_t>::max()) {
            ++summary_.faults; return;
        }
        ++generation_; serial_=0; lastSubmit_.reset(); lastBoundary_=e.atUs;
        summary_.connected=true;
        connection_=UsbHostConnection{generation_,e.atUs,true};
        return;
    }
    if (e.kind==UsbHostEventKind::Detached) {
        if (e.attachment<attachment_) { ++summary_.rejectedEvents; return; }
        attachment_=e.attachment;
        loseConnection(e.atUs);
        return;
    }
    if (e.kind==UsbHostEventKind::Error && !e.token.generation && !e.token.serial &&
        e.attachment>=attachment_) {
        attachment_=e.attachment; ++summary_.faults; loseConnection(e.atUs); return;
    }
    if (e.attachment!=attachment_ || !summary_.connected || !summary_.inFlight ||
        e.token.generation!=generation_ || e.token.serial!=serial_) {
        ++summary_.rejectedEvents; return;
    }
    summary_.inFlight=false;
    // NAKも新規の操作報告ではない。長押しと中立復帰履歴を維持しない。
    if (e.atUs<submittedAt_ || e.atUs<lastBoundary_ ||
        (serial_>1u && e.atUs==lastBoundary_) ||
        nowUs-submittedAt_>=config_.transferTimeoutUs ||
        e.kind==UsbHostEventKind::Stall || e.kind==UsbHostEventKind::Error) {
        ++summary_.faults; loseConnection(nowUs); return;
    }
    lastBoundary_=e.atUs;
    UsbHostReport report{};
    report.token=e.token; report.receivedAtUs=e.atUs;
    if (e.kind==UsbHostEventKind::Done && e.length==config_.reportLength) {
        report.length=e.length; report.data=e.data; ++summary_.reports;
    } else {
        // 不正長/NAKは長さ0で一度だけ配送し、既存HID層に連続性を無効化させる。
        if (e.kind==UsbHostEventKind::Nak) ++summary_.naks;
        else ++summary_.invalidReports;
    }
    report_=report;
}

void UsbHostPort::poll(std::uint64_t nowUs) noexcept
{
    if (!summary_.started) return;
    // 未回収の固定一件を上書きしない。Applicationは毎poll後に両通知を回収する。
    if (connection_ || report_) return;
    if (lastPoll_ && nowUs<*lastPoll_) {
        ++summary_.faults; loseConnection(*lastPoll_); return;
    }
    lastPoll_=nowUs;
    handle(backend_.poll(backend_.context,nowUs),nowUs);
    if (!summary_.connected || connection_ || report_) return;
    if (summary_.inFlight) {
        if (nowUs-submittedAt_>=config_.transferTimeoutUs) {
            ++summary_.faults; loseConnection(nowUs);
        }
        return;
    }
    if (lastSubmit_ && nowUs-*lastSubmit_<static_cast<std::uint64_t>(config_.intervalMs)*1000u) return;
    if (serial_==std::numeric_limits<std::uint64_t>::max()) {
        ++summary_.faults; loseConnection(nowUs); return;
    }
    const UsbTransferToken token{generation_,++serial_};
    submittedAt_=nowUs; lastSubmit_=nowUs;
    // 下位は受付のみ。完了通知が来るまで同じバッファを再使用しない。
    if (backend_.submit(backend_.context,token,config_.packetBytes)) summary_.inFlight=true;
    else { ++summary_.faults; loseConnection(nowUs); }
}

std::optional<UsbHostConnection> UsbHostPort::takeConnectionEvent() noexcept
{ const auto value=connection_; connection_.reset(); return value; }
std::optional<UsbHostReport> UsbHostPort::takeReport() noexcept
{ const auto value=report_; report_.reset(); return value; }
}
