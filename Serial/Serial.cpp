#include "Serial.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace TRoMaC
{
namespace
{
speed_t baudRate(int value) noexcept
{
    switch (value)
    {
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 921600: return B921600;
        case 1000000: return B1000000;
        default: return 0;
    }
}

bool writeAll(int fd, const uint8_t* bytes, std::size_t size) noexcept
{
    if (fd < 0 || (bytes == nullptr && size != 0)) return false;
    std::size_t offset = 0;
    while (offset < size)
    {
        const ssize_t count = write(fd, bytes + offset, size - offset);
        if (count > 0) offset += static_cast<std::size_t>(count);
        else if (count < 0 && errno == EINTR) continue;
        else return false;
    }
    return true;
}

void writeFloatLittleEndian(uint8_t* destination, float value) noexcept
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    destination[0] = static_cast<uint8_t>(bits & 0xFFU);
    destination[1] = static_cast<uint8_t>((bits >> 8) & 0xFFU);
    destination[2] = static_cast<uint8_t>((bits >> 16) & 0xFFU);
    destination[3] = static_cast<uint8_t>(bits >> 24);
}

float readFloatLittleEndian(const uint8_t* source) noexcept
{
    const uint32_t bits = static_cast<uint32_t>(source[0]) |
                          (static_cast<uint32_t>(source[1]) << 8) |
                          (static_cast<uint32_t>(source[2]) << 16) |
                          (static_cast<uint32_t>(source[3]) << 24);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void writeUint16LittleEndian(uint8_t* destination, uint16_t value) noexcept
{
    destination[0] = static_cast<uint8_t>(value & 0xFFU);
    destination[1] = static_cast<uint8_t>(value >> 8);
}

uint16_t clampWheelSpeed(uint16_t value) noexcept
{
    return static_cast<uint16_t>(std::min<unsigned int>(value, 800U));
}

int openDevice(int comport, const char*& path) noexcept
{
    switch (comport)
    {
        case 1: path = "/dev/ttyUSB_vision"; break;
        case 2: path = "/dev/ttyS1"; break;
        case 3: path = "/dev/ttyS2"; break;
        default: path = nullptr; return -1;
    }
    return open(path, O_RDWR | O_NOCTTY | O_NDELAY);
}
} // namespace

RxBase::RxBase() noexcept
    : state_(RxParseState::Header), payload_{}, command_(0), payloadIndex_(0), ready_(false)
{
}

bool RxBase::consume(uint8_t byte)
{
    switch (state_)
    {
        case RxParseState::Header:
            if (byte == kFrameHeader)
            {
                ready_ = false;
                state_ = RxParseState::Command;
            }
            break;
        case RxParseState::Command:
            // The MCU feedback direction only uses command 0x01.  Command
            // 0x02 is a host-to-MCU incremental command and must not be
            // accepted as an incoming feedback frame.
            if (byte != kVisionCommand)
            {
                state_ = byte == kFrameHeader ? RxParseState::Command : RxParseState::Header;
                break;
            }
            command_ = byte;
            payloadIndex_ = 0;
            state_ = RxParseState::Data;
            break;
        case RxParseState::Data:
            payload_[payloadIndex_++] = byte;
            if (payloadIndex_ == kFeedbackPayloadSize)
                state_ = RxParseState::Tail;
            break;
        case RxParseState::Tail:
            return finishFrame(byte);
    }
    return false;
}

bool RxBase::finishFrame(uint8_t tailByte)
{
    const bool valid = tailByte == kFrameTail;
    if (!valid)
        std::cerr << "接收错误：帧尾错误，收到 0x" << std::hex
                  << static_cast<int>(tailByte) << "，期望 0x"
                  << static_cast<int>(kFrameTail) << std::dec << '\n';
    ready_ = valid;
    resetParser();
    // A5 can already be the first byte of the next fixed frame when a byte is
    // dropped. Preserve it as a new header instead of discarding it.
    if (!valid && tailByte == kFrameHeader)
        state_ = RxParseState::Command;
    return valid;
}

bool RxBase::ready() const noexcept { return ready_; }
uint8_t RxBase::command() const noexcept { return command_; }
std::size_t RxBase::payloadSize() const noexcept { return kFeedbackPayloadSize; }
const std::array<uint8_t, kFeedbackPayloadSize>& RxBase::payload() const noexcept { return payload_; }

void RxBase::clear() noexcept
{
    payload_.fill(0);
    command_ = 0;
    ready_ = false;
    resetParser();
}

void RxBase::resetParser() noexcept
{
    state_ = RxParseState::Header;
    payloadIndex_ = 0;
}

TxCommand01::TxCommand01(const VisionData& data) noexcept
{
    const uint8_t command = data.Gimbal_Ctrl == kIncrementalCommand
        ? kIncrementalCommand : kAbsoluteCommand;
    uint16_t left = clampWheelSpeed(data.left_speed);
    uint16_t right = clampWheelSpeed(data.right_speed);
    if (std::abs(static_cast<int>(left) - static_cast<int>(right)) > 100)
        left = right = 660;

    bytes_.fill(0);
    bytes_[0] = kFrameHeader;
    bytes_[1] = command;
    writeFloatLittleEndian(bytes_.data() + 2,
        command == kAbsoluteCommand ? data.pitch_angle : data.pitch_delta_angle);
    writeFloatLittleEndian(bytes_.data() + 6,
        command == kAbsoluteCommand ? data.yaw_angle : data.yaw_delta_angle);
    writeUint16LittleEndian(bytes_.data() + 10, left);
    writeUint16LittleEndian(bytes_.data() + 12, right);
    bytes_[14] = data.buzzer ? 1 : 0;
    bytes_[15] = data.Push_flag ? 1 : 0;
    bytes_[16] = kFrameTail;
}

bool TxCommand01::writeTo(int fd) const noexcept
{
    return writeAll(fd, bytes_.data(), bytes_.size());
}

Uart::Uart() noexcept : serialId_(-1), speed_(0), devicePath_(), rx_{} {}
Uart::~Uart() noexcept { Close(); }

Uart::Uart(Uart&& other) noexcept
    : serialId_(other.serialId_), speed_(other.speed_),
      devicePath_(std::move(other.devicePath_)), rx_(other.rx_)
{
    other.serialId_ = -1;
    other.speed_ = 0;
    other.rx_.clear();
}

Uart& Uart::operator=(Uart&& other) noexcept
{
    if (this != &other)
    {
        Close();
        serialId_ = other.serialId_;
        speed_ = other.speed_;
        devicePath_ = std::move(other.devicePath_);
        rx_ = other.rx_;
        other.serialId_ = -1;
        other.speed_ = 0;
        other.rx_.clear();
    }
    return *this;
}

void Uart::Close() noexcept
{
    if (serialId_ >= 0) close(serialId_);
    serialId_ = -1;
    speed_ = 0;
    devicePath_.clear();
    rx_.clear();
}

bool Uart::IsOpen() const noexcept { return serialId_ >= 0; }
const std::string& Uart::devicePath() const noexcept { return devicePath_; }

int Uart::open_port(int comport)
{
    const char* path = nullptr;
    const int fd = openDevice(comport, path);
    if (fd < 0)
    {
        if (path == nullptr) std::cerr << "无效串口编号: " << comport << '\n';
        else std::cerr << "无法打开串口 " << path << ": " << std::strerror(errno) << '\n';
        return -1;
    }
    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        close(fd);
        return -1;
    }
    devicePath_ = path;
    return fd;
}

int Uart::set_opt(int fd, int baud, int bits, char parity, int stopBits)
{
    if (fd < 0 || bits != 8 || (stopBits != 1 && stopBits != 2)) return -1;
    const speed_t rate = baudRate(baud);
    if (rate == 0) return -1;
    termios options{};
    if (tcgetattr(fd, &options) != 0) return -1;
    cfmakeraw(&options);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CSTOPB;
    if (stopBits == 2) options.c_cflag |= CSTOPB;
    options.c_cflag &= ~(PARENB | PARODD);
    if (parity == 'e' || parity == 'E') options.c_cflag |= PARENB;
    else if (parity == 'o' || parity == 'O') options.c_cflag |= PARENB | PARODD;
    else if (parity != 'n' && parity != 'N') return -1;
    cfsetispeed(&options, rate);
    cfsetospeed(&options, rate);
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    return tcsetattr(fd, TCSANOW, &options) == 0 ? 0 : -1;
}

bool Uart::Open(int comport, int baud)
{
    Close();
    const int fd = open_port(comport);
    if (fd < 0 || set_opt(fd, baud, 8, 'N', 1) != 0)
    {
        if (fd >= 0) close(fd);
        devicePath_.clear();
        return false;
    }
    serialId_ = fd;
    speed_ = baud;
    return true;
}

bool Uart::checkSerial()
{
    if (!IsOpen()) return false;
    termios options{};
    if (tcgetattr(serialId_, &options) == 0) return true;
    Close();
    return false;
}

bool Uart::ReadData()
{
    if (!IsOpen()) return false;
    uint8_t bytes[64]{};
    ssize_t count;
    do { count = read(serialId_, bytes, sizeof(bytes)); }
    while (count < 0 && errno == EINTR);
    if (count <= 0) return false;
    bool receivedFrame = false;
    for (ssize_t i = 0; i < count; ++i)
        receivedFrame = ProtocolParse(bytes[i]) || receivedFrame;
    return receivedFrame;
}

bool Uart::send(const VisionData& data)
{
    const TxCommand01 frame(data);
    return frame.writeTo(serialId_);
}

bool Uart::ProtocolParse(uint8_t byte) { return rx_.consume(byte); }
const RxBase& Uart::lastRxFrame() const noexcept { return rx_; }

bool Uart::tryGetGimbalFeedback(GimbalFeedback& output) const noexcept
{
    if (!rx_.ready() || rx_.command() != kAbsoluteCommand) return false;
    const auto& payload = rx_.payload();
    output.pitch_angle = readFloatLittleEndian(payload.data());
    output.yaw_angle = readFloatLittleEndian(payload.data() + 4);
    output.left_speed = static_cast<uint16_t>(payload[8]) |
                        static_cast<uint16_t>(payload[9] << 8);
    output.right_speed = static_cast<uint16_t>(payload[10]) |
                         static_cast<uint16_t>(payload[11] << 8);
    return std::isfinite(output.yaw_angle) && std::isfinite(output.pitch_angle);
}

} // namespace TRoMaC
