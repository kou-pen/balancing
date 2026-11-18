#pragma once

namespace balancing {

/** @brief ロボットの状態を保持する小さな共通基底。
 *  @spec{BEHAV-001}
 */
template <typename State>
class StateMachine {
public:
    /** @brief 現在の状態を返す。 @spec{BEHAV-001} */
    constexpr State state() const noexcept
    {
        return state_;
    }

protected:
    constexpr explicit StateMachine(State initial) noexcept : state_{initial} {}

    constexpr void transitionTo(State next) noexcept
    {
        state_ = next;
    }

private:
    State state_;
};

} // namespace balancing
