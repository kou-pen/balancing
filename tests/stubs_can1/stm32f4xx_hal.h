#pragma once

#include <cstdint>
#include <initializer_list>

constexpr std::uint32_t CAN_TSR_RQCP0 = 1u;
constexpr std::uint32_t CAN_TSR_TXOK0 = 2u;
constexpr std::uint32_t CAN_TSR_ALST0 = 4u;
constexpr std::uint32_t CAN_TSR_TERR0 = 8u;
constexpr std::uint32_t CAN_TSR_ABRQ0 = 0x80u;
constexpr std::uint32_t CAN_TSR_TME0 = 1u << 26u;
constexpr std::uint32_t CAN_TI0R_IDE = 4u;
constexpr std::uint32_t CAN_TI0R_TXRQ = 1u;
constexpr std::uint32_t CAN_RI0R_IDE = 4u;
constexpr std::uint32_t CAN_RI0R_RTR = 2u;
constexpr std::uint32_t CAN_RDT0R_DLC = 0xfu;
constexpr std::uint32_t CAN_RF0R_FMP0 = 3u;
constexpr std::uint32_t CAN_RF0R_FOVR0 = 0x10u;
constexpr std::uint32_t CAN_RF0R_RFOM0 = 0x20u;
constexpr std::uint32_t CAN_ESR_BOFF = 4u;
constexpr std::uint32_t CAN_FLAG_RQCP0 = 0x500u;
constexpr std::uint32_t CAN_FLAG_RQCP1 = 0x508u;
constexpr std::uint32_t CAN_FLAG_RQCP2 = 0x510u;
constexpr std::uint32_t CAN_FLAG_FOV0 = 0x204u;
constexpr std::uint32_t CAN_FLAG_ERRI = 0x102u;

struct TestTxStatus {
    std::uint32_t bits{7u << 26u};
    std::uint32_t aborts{0};
    operator std::uint32_t() const noexcept { return bits; }
    TestTxStatus& operator=(std::uint32_t written) noexcept
    {
        // bxCANは完了ビットへの1の書き込みでその枠の終端フラグを消す。
        for (unsigned shift : {0u, 8u, 16u}) {
            if ((written & (CAN_TSR_RQCP0 << shift)) != 0u) {
                bits &= ~(0xfu << shift);
            }
        }
        aborts |= written & 0x00808080u;
        return *this;
    }
};

struct TestRxMailbox { std::uint32_t RIR, RDTR, RDLR, RDHR; };

struct TestRxStatus {
    TestRxMailbox* frames;
    std::uint32_t count{0};
    std::uint32_t flags{0};
    operator std::uint32_t() const noexcept { return count | flags; }
    TestRxStatus& operator=(std::uint32_t written) noexcept
    {
        if ((written & CAN_RF0R_RFOM0) != 0u && count > 0) {
            --count;
            for (std::uint32_t i = 0; i < count; ++i) frames[i] = frames[i + 1u];
        }
        flags &= ~(written & CAN_RF0R_FOVR0);
        return *this;
    }
};

struct CAN_TypeDef {
    TestTxStatus TSR{};
    std::uint32_t ESR{0};
    std::uint32_t MSR{0};
    struct { std::uint32_t TIR, TDTR, TDLR, TDHR; } sTxMailBox[3]{};
    TestRxMailbox sFIFOMailBox[3]{};
    TestRxStatus RF0R{sFIFOMailBox};
};

enum HAL_CAN_StateTypeDef { HAL_CAN_STATE_READY = 1, HAL_CAN_STATE_LISTENING = 2, HAL_CAN_STATE_ERROR = 3 };
struct CAN_HandleTypeDef { CAN_TypeDef* Instance; HAL_CAN_StateTypeDef State; };

inline HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef* h) { return h->State; }
inline std::uint32_t __HAL_CAN_GET_FLAG(CAN_HandleTypeDef* h, std::uint32_t flag)
{
    return flag == CAN_FLAG_FOV0 ? (h->Instance->RF0R & CAN_RF0R_FOVR0) : (h->Instance->MSR & 4u);
}
inline void __HAL_CAN_CLEAR_FLAG(CAN_HandleTypeDef* h, std::uint32_t flag)
{
    if (flag == CAN_FLAG_FOV0) h->Instance->RF0R = CAN_RF0R_FOVR0;
    else if (flag == CAN_FLAG_ERRI) h->Instance->MSR &= ~4u;
    else h->Instance->TSR = 1u << (flag & 31u);
}

std::uint32_t __get_PRIMASK();
void __disable_irq();
void __enable_irq();


constexpr std::uint32_t CAN_IT_TX_MAILBOX_EMPTY = 0x1u;
constexpr std::uint32_t CAN_IT_RX_FIFO0_MSG_PENDING = 0x2u;
constexpr std::uint32_t CAN_IT_RX_FIFO0_OVERRUN = 0x8u;
constexpr std::uint32_t CAN_IT_BUSOFF = 0x400u;
constexpr std::uint32_t CAN_IT_ERROR = 0x8000u;
constexpr std::uint32_t CAN_FILTERMODE_IDMASK = 0u;
constexpr std::uint32_t CAN_FILTERSCALE_32BIT = 1u;
constexpr std::uint32_t CAN_FILTER_FIFO0 = 0u;
constexpr std::uint32_t ENABLE = 1u;
constexpr std::uint32_t CoreDebug_DEMCR_TRCENA_Msk = 1u << 24u;
constexpr std::uint32_t DWT_CTRL_NOCYCCNT_Msk = 1u << 25u;
constexpr std::uint32_t DWT_CTRL_CYCCNTENA_Msk = 1u;

struct CAN_FilterTypeDef {
    std::uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
    std::uint32_t FilterFIFOAssignment, FilterBank, FilterMode, FilterScale;
    std::uint32_t FilterActivation, SlaveStartFilterBank;
};
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY };
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, const CAN_FilterTypeDef*);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, std::uint32_t);
void __HAL_CAN_DISABLE_IT(CAN_HandleTypeDef*, std::uint32_t);
void __set_PRIMASK(std::uint32_t);

struct TestDwtRegisters { std::uint32_t CTRL, CYCCNT; };
struct TestCoreDebugRegisters { std::uint32_t DEMCR; };
extern TestDwtRegisters* DWT;
extern TestCoreDebugRegisters* CoreDebug;
extern std::uint32_t SystemCoreClock;
extern CAN_TypeDef* CAN1;
extern CAN_TypeDef* CAN2;

struct I2C_TypeDef { std::uint32_t SR2{0}; };
enum HAL_I2C_StateTypeDef { HAL_I2C_STATE_READY, HAL_I2C_STATE_BUSY };
struct I2C_HandleTypeDef { I2C_TypeDef* Instance; HAL_I2C_StateTypeDef State; };
inline HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef* h) { return h->State; }
constexpr std::uint32_t I2C_SR2_BUSY=2u;
constexpr std::uint16_t I2C_MEMADD_SIZE_8BIT=1u;
constexpr int I2C1_EV_IRQn=31,I2C1_ER_IRQn=32;
extern I2C_TypeDef* I2C1;
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef*,std::uint16_t,std::uint16_t,std::uint16_t,std::uint8_t*,std::uint16_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef*,std::uint16_t,std::uint16_t,std::uint16_t,std::uint8_t*,std::uint16_t);
void HAL_NVIC_SetPriority(int,int,int);
void HAL_NVIC_EnableIRQ(int);
