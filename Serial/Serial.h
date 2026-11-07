#ifndef TROMAC_SERIAL_H_
#define TROMAC_SERIAL_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace TRoMaC
{

constexpr uint8_t kFrameHeader = 0xA5;
constexpr uint8_t kFrameTail = 0x5A;
constexpr uint8_t kAbsoluteCommand = 0x01;
constexpr uint8_t kIncrementalCommand = 0x02;
constexpr uint8_t kVisionCommand = kAbsoluteCommand;
constexpr uint8_t kControllerCommand = kIncrementalCommand;
constexpr std::size_t kFeedbackPayloadSize = 12;
constexpr std::size_t kFeedbackFrameSize = 15;
constexpr std::size_t kCommandFrameSize = 17;

#pragma pack(push, 1)
struct GimbalFeedback
{
    float pitch_angle;
    float yaw_angle;
    uint16_t left_speed;
    uint16_t right_speed;
};

// Gimbal_Ctrl: 1=absolute, 2=incremental. This is a semantic structure;
// the wire frame is always serialized explicitly in Serial.cpp.
struct VisionData
{
    uint8_t Gimbal_Ctrl;
    float pitch_angle;
    float yaw_angle;
    float pitch_delta_angle;
    float yaw_delta_angle;
    uint16_t left_speed;
    uint16_t right_speed;
    uint8_t buzzer;
    uint8_t Push_flag;
};
#pragma pack(pop)

static_assert(sizeof(float) == 4, "Protocol requires a 4-byte float");
static_assert(sizeof(GimbalFeedback) == 12, "Feedback payload size changed");
static_assert(sizeof(VisionData) == 23, "VisionData semantic layout changed");

enum class RxParseState : uint8_t
{
    Header,
    Command,
    Data,
    Tail
};

class RxBase
{
public:
    RxBase() noexcept;
    RxBase(const RxBase&) = default;
    RxBase& operator=(const RxBase&) = default;

    bool consume(uint8_t byte);
    bool ready() const noexcept;
    uint8_t command() const noexcept;
    std::size_t payloadSize() const noexcept;
    const std::array<uint8_t, kFeedbackPayloadSize>& payload() const noexcept;
    void clear() noexcept;

private:
    void resetParser() noexcept;
    bool finishFrame(uint8_t tailByte);

    RxParseState state_;
    std::array<uint8_t, kFeedbackPayloadSize> payload_;
    uint8_t command_;
    std::size_t payloadIndex_;
    bool ready_;
};

class TxCommand01
{
public:
    explicit TxCommand01(const VisionData& data) noexcept;
    const uint8_t* bytes() const noexcept { return bytes_.data(); }
    std::size_t size() const noexcept { return bytes_.size(); }
    bool writeTo(int fd) const noexcept;

private:
    std::array<uint8_t, kCommandFrameSize> bytes_{};
};

class Uart
{
public:
    Uart() noexcept;
    ~Uart() noexcept;

    Uart(const Uart&) = delete;
    Uart& operator=(const Uart&) = delete;
    Uart(Uart&& other) noexcept;
    Uart& operator=(Uart&& other) noexcept;

    void Close() noexcept;
    bool IsOpen() const noexcept;
    const std::string& devicePath() const noexcept;

    int open_port(int comport);
    int set_opt(int fd, int speed, int bits, char parity, int stopBits);
    bool Open(int comport, int speed);
    bool checkSerial();
    bool ReadData();
    bool send(const VisionData& data);
    bool ProtocolParse(uint8_t byte);

    const RxBase& lastRxFrame() const noexcept;
    bool tryGetGimbalFeedback(GimbalFeedback& output) const noexcept;

private:
    int serialId_;
    int speed_;
    std::string devicePath_;
    RxBase rx_;
};

} // namespace TRoMaC

#endif // TROMAC_SERIAL_H_
