#ifndef PARAMETER_ROUTER_H
#define PARAMETER_ROUTER_H

#include <cstdint>

class ClosedLoopController;

// 把上位机 / TMC 兼容寄存器访问翻译成控制器动作与参数读写。
//
// 这两段 if-else 链原先直接长在 ClosedLoopController 里（写 241 行 / 读 265 行），
// 占了文件近 1/5，而且和运动控制毫无关系 —— 读 rampUpdate 时要先跳过几百行寄存器映射。
// 现在集中到这里，控制器只留一行的转发入口。
//
// 实现上是控制器的 friend：参数本来就分散在运动状态、PID、编码器、负载角等多个子模块里，
// 强行给每一项都开一个 public 访问器，只会把控制器的接口撑得比现在更难看清。
class ParameterRouter
{
public:
    explicit ParameterRouter(ClosedLoopController& owner);

    // 写一个扩展寄存器。返回 false 表示这张表里不认识这个地址（会继续往下走
    // 到协议层的官方 TMC 寄存器分支）。
    bool write(uint16_t reg, uint32_t value);

    // 读一个扩展寄存器。
    bool read(uint16_t reg, uint32_t* value);

private:
    ClosedLoopController& owner_;
};

#endif
