#include "balancing/device/StatusIndicatorPort.hpp"
#if defined(__arm__) || defined(__thumb__)
#include "main.h"
namespace balancing {
namespace {
struct GpioPin { GPIO_TypeDef* port; std::uint16_t pin; };
bool writeGpio(void* ctx,bool high) noexcept {
    const auto& p=*static_cast<GpioPin*>(ctx);
    HAL_GPIO_WritePin(p.port,p.pin,high?GPIO_PIN_SET:GPIO_PIN_RESET);
    // ODRの照合は出力ラッチだけの検査。LEDの発光・実端子電圧や鳴動の証明ではない。
    return bool(p.port->ODR&p.pin)==high;
}
}
IndicatorOutputConfig makeF446IndicatorOutput(const std::array<bool,4>& polarity,bool verified) noexcept {
    // CubeMXの生成定義を唯一のピン割当元とする。LED1〜5の用途は設定しない。
    static GpioPin pins[]={{LED_R_GPIO_Port,LED_R_Pin},{LED_G_GPIO_Port,LED_G_Pin},
        {LED_B_GPIO_Port,LED_B_Pin},{BUZZER_GPIO_Port,BUZZER_Pin}};
    IndicatorOutputConfig result{};
    result.verified=verified;
    for (std::size_t i=0;i<4;++i) result.pins[i]={nullptr,&pins[i],polarity[i],verified,writeGpio};
    return result;
}
}
#endif
