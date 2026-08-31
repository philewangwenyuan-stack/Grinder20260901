#include <windows.h>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// ============================================================
// 标准 Modbus RTU 双驱动差速底盘控制程序（最小必要保护版）
//
// 本程序用途：
// 1. 从控制台读取线速度 v 和角速度 w；
// 2. 根据差速底盘参数，把 v/w 换算成左右轮目标转速；
// 3. 通过 Modbus RTU 把左右轮目标值写到寄存器；
// 4. 读回命令回显，必要时可扩展读取真实反馈；
// 5. 在运行过程中增加 6 类最小必要保护：
//    - 输入限幅
//    - 斜坡限制
//    - 启动零输出
//    - 退出零输出
//    - 异常自动停车
//    - 连续通信/回显异常保护
//
// 适用场景：
// - 已完成基础双驱动 Modbus 通信调通；
// - 希望在现场联调阶段加入最基本安全保护；
// - 仍然使用控制台手动输入 v/w 做测试。
// ============================================================

// Modbus 标准功能码：
// 0x03 = 读保持寄存器
// 0x06 = 写单个保持寄存器
// 0x10 = 写多个保持寄存器
constexpr uint8_t FC_READ_HOLDING_REGS = 0x03;
constexpr uint8_t FC_WRITE_SINGLE_REG = 0x06;
constexpr uint8_t FC_WRITE_MULTI_REGS = 0x10;

// 写入模式切换：
// true  -> 使用 0x06 单寄存器写
// false -> 使用 0x10 多寄存器写
constexpr bool USE_SINGLE_REGISTER_WRITE = true;

// 最小必要保护参数：
// 这一组参数决定程序在“现场调试 / 保守运行”时的安全边界。
// 如果你希望车子动作更温和，就把这些值调小；
// 如果已经联调稳定、想提高响应速度，再逐步调大。
//
// MAX_INPUT_V：
// - 控制台允许输入的线速度上限，单位通常是 m/s。
// - 例如当前 0.083 表示，输入 0.1 也会被截断成 0.083。
// - 这个值越小，前进/后退越保守。
constexpr double MAX_INPUT_V = 0.083;

// MAX_INPUT_W：
// - 控制台允许输入的角速度上限，单位通常是 rad/s。
// - 这个值越小，原地转向或转弯动作越慢、越柔和。
constexpr double MAX_INPUT_W = 0.1;

// MAX_ABS_WHEEL_RPM：
// - 左右轮目标转速在写入寄存器前的最终限幅值，单位 rpm。
// - 即使根据 v/w 换算出的轮速更高，也会被截断到这个范围内。
// - 用来防止因为参数配置或输入过大导致电机转速命令过猛。
constexpr double MAX_ABS_WHEEL_RPM = 1500.0;

// MAX_CMD_STEP_RPM：
// - 斜坡限制参数，表示每次循环中命令最多允许变化多少 rpm。
// - 例如上一拍是 0，本拍目标是 200，而这里设为 50，
//   那么本次只会先发到 50，不会直接跳到 200。
// - 这个值越小，速度变化越平滑，但响应也会更慢。
constexpr int16_t MAX_CMD_STEP_RPM = 50;

// MAX_COMM_FAILURE_COUNT：
// - 连续通信失败多少次后，触发自动停车保护。
// - 设得小：更安全，但偶发通信抖动也更容易停车。
// - 设得大：抗偶发抖动更强，但故障停机反应会变慢。
constexpr int MAX_COMM_FAILURE_COUNT = 2;

// MAX_ECHO_DEVIATION：
// - 命令值与读回回显值之间允许的最大差值。
// - 如果设备内部会做滤波、限幅或换算，这个值通常不能设得太小。
// - 设得太小可能会频繁误报“回显偏差异常”。
constexpr int MAX_ECHO_DEVIATION = 200;

// MAX_ECHO_FAILURE_COUNT：
// - 连续多少次回显偏差超限后，触发自动停车。
// - 当前值 2 表示：只要连续两次超差，就认为状态异常并停车。
constexpr int MAX_ECHO_FAILURE_COUNT = 2;

// 目标速度寄存器与反馈寄存器：
// 左右命令寄存器已确定；实际反馈寄存器未知时保持 0xFFFF。
constexpr uint16_t REG_LEFT_CMD = 0x4010;
constexpr uint16_t REG_RIGHT_CMD = 0x4011;
constexpr uint16_t REG_LEFT_ACT = 0xFFFF;
constexpr uint16_t REG_RIGHT_ACT = 0xFFFF;

struct RobotParams {
    // 轮半径、轮距、减速比与方向符号共同决定 v/w 到左右轮 rpm 的换算结果
    double wheelRadiusM = 0.30;
    double wheelBaseM = 0.79;
    double gearRatio = 60.0;
    int leftDirSign = 1;
    int rightDirSign = 1;
};

struct WheelSpeedRpm {
    // 左右轮的浮点 rpm 值，便于先做运动学换算，再做限幅
    double left = 0.0;
    double right = 0.0;
};

struct WheelSpeedI16 {
    // 最终真正写入寄存器的 int16 值
    int16_t left = 0;
    int16_t right = 0;
};

static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
    // 标准 Modbus RTU CRC16 计算函数
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; ++pos) {
        crc ^= data[pos];
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

static std::string hexDump(const std::vector<uint8_t>& data) {
    // 把原始字节帧转成十六进制字符串，便于调试串口收发过程
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < data.size(); ++i) {
        oss << std::setw(2) << static_cast<unsigned>(data[i]);
        if (i + 1 != data.size()) oss << ' ';
    }
    return oss.str();
}

static int16_t clampToI16(double x) {
    // 把浮点速度值压到 int16 范围，确保可写入 16 位寄存器
    if (x > 32767.0) x = 32767.0;
    if (x < -32768.0) x = -32768.0;
    return static_cast<int16_t>(std::lround(x));
}

static WheelSpeedRpm calcWheelSpeedRpm(double v, double w, const RobotParams& p) {
    // 差速底盘运动学：v/w -> 左右轮线速度 -> 左右轮 rpm
    const double vL = v - w * p.wheelBaseM / 2.0;
    const double vR = v + w * p.wheelBaseM / 2.0;
    const double rpmL = (vL / p.wheelRadiusM) * 60.0 / (2.0 * 3.14159265358979323846);
    const double rpmR = (vR / p.wheelRadiusM) * 60.0 / (2.0 * 3.14159265358979323846);
    return {
        rpmL * p.gearRatio * static_cast<double>(p.leftDirSign),
        rpmR * p.gearRatio * static_cast<double>(p.rightDirSign)
    };
}

static WheelSpeedI16 toI16(const WheelSpeedRpm& rpm, double maxAbsWheelRpm) {
    // 先做业务层轮速限幅，再转成 int16 寄存器值
    WheelSpeedRpm limited = rpm;
    if (limited.left > maxAbsWheelRpm) limited.left = maxAbsWheelRpm;
    if (limited.left < -maxAbsWheelRpm) limited.left = -maxAbsWheelRpm;
    if (limited.right > maxAbsWheelRpm) limited.right = maxAbsWheelRpm;
    if (limited.right < -maxAbsWheelRpm) limited.right = -maxAbsWheelRpm;
    return {clampToI16(limited.left), clampToI16(limited.right)};
}

class SerialPort {
public:
    // 串口对象析构时自动释放句柄
    ~SerialPort() {
        if (handle_ != INVALID_HANDLE_VALUE) CloseHandle(handle_);
    }

    // 按当前程序设定的串口参数打开串口
    void openPort(const std::string& portName, DWORD baudRate,
                  BYTE byteSize = 8, BYTE parity = NOPARITY, BYTE stopBits = ONESTOPBIT) {
        std::string realPort = portName;
        if (realPort.rfind("\\\\.\\", 0) != 0) realPort = "\\\\.\\" + realPort;

        handle_ = CreateFileA(realPort.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                              nullptr, OPEN_EXISTING, 0, nullptr);
        if (handle_ == INVALID_HANDLE_VALUE) throw std::runtime_error("打开串口失败: " + portName);

        DCB dcb{};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(handle_, &dcb)) throw std::runtime_error("GetCommState 失败");

        dcb.BaudRate = baudRate;
        dcb.ByteSize = byteSize;
        dcb.Parity = parity;
        dcb.StopBits = stopBits;
        dcb.fBinary = TRUE;
        dcb.fParity = (parity == NOPARITY) ? FALSE : TRUE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fTXContinueOnXoff = TRUE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fErrorChar = FALSE;
        dcb.fNull = FALSE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fAbortOnError = FALSE;
        if (!SetCommState(handle_, &dcb)) throw std::runtime_error("SetCommState 失败");

        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout = 20;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutConstant = 50;
        if (!SetCommTimeouts(handle_, &timeouts)) throw std::runtime_error("SetCommTimeouts 失败");

        SetupComm(handle_, 4096, 4096);
        PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    }

    void flush() { PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR); }

    void writeAll(const std::vector<uint8_t>& data) {
        DWORD totalWritten = 0;
        while (totalWritten < data.size()) {
            DWORD written = 0;
            if (!WriteFile(handle_, data.data() + totalWritten,
                           static_cast<DWORD>(data.size() - totalWritten),
                           &written, nullptr)) {
                throw std::runtime_error("WriteFile 失败");
            }
            totalWritten += written;
        }
        FlushFileBuffers(handle_);
    }

    std::vector<uint8_t> readSome(size_t expectedMin, int timeoutMs = 100) {
        std::vector<uint8_t> buffer;
        auto start = GetTickCount64();
        while (true) {
            uint8_t tmp[256];
            DWORD bytesRead = 0;
            if (!ReadFile(handle_, tmp, sizeof(tmp), &bytesRead, nullptr)) {
                throw std::runtime_error("ReadFile 失败");
            }
            if (bytesRead > 0) {
                buffer.insert(buffer.end(), tmp, tmp + bytesRead);
                if (buffer.size() >= expectedMin) return buffer;
            }
            if (static_cast<int>(GetTickCount64() - start) >= timeoutMs) return buffer;
            Sleep(1);
        }
    }

private:
    HANDLE handle_ = INVALID_HANDLE_VALUE;
};

class ModbusRtuMaster {
public:
    // 在 SerialPort 基础上封装 Modbus RTU 的拼帧、CRC、收发和解析
    explicit ModbusRtuMaster(SerialPort& port) : port_(port) {}

    std::vector<uint16_t> readHoldingRegisters(uint8_t slave, uint16_t reg, uint16_t count,
                                               const char* tag = nullptr) {
        std::vector<uint8_t> req = {
            slave, FC_READ_HOLDING_REGS,
            static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg),
            static_cast<uint8_t>(count >> 8), static_cast<uint8_t>(count)
        };
        appendCrc(req);

        if (tag && *tag) std::cout << '[' << tag << "] ";
        std::cout << "TX(0x03): slave=" << static_cast<int>(slave)
                  << " startReg=0x" << std::hex << std::setw(4) << std::setfill('0') << reg
                  << std::dec << " count=" << count << " frame=[" << hexDump(req) << "]\n";

        port_.flush();
        port_.writeAll(req);

        auto resp = port_.readSome(5, 100);
        handleExceptionIfNeeded(slave, FC_READ_HOLDING_REGS, resp);

        const size_t expected = 5 + static_cast<size_t>(count) * 2;
        if (resp.size() < expected) {
            auto extra = port_.readSome(expected, 50);
            resp.insert(resp.end(), extra.begin(), extra.end());
        }
        if (resp.size() != expected) {
            throw std::runtime_error("读保持寄存器响应长度异常: " + hexDump(resp));
        }

        verifyCrc(resp);
        if (resp[0] != slave || resp[1] != FC_READ_HOLDING_REGS || resp[2] != count * 2) {
            throw std::runtime_error("读保持寄存器响应异常: " + hexDump(resp));
        }

        std::vector<uint16_t> regs;
        for (uint16_t i = 0; i < count; ++i) {
            regs.push_back(static_cast<uint16_t>(resp[3 + 2 * i] << 8) |
                           static_cast<uint16_t>(resp[4 + 2 * i]));
        }

        if (tag && *tag) std::cout << '[' << tag << "] ";
        std::cout << "RX(0x03): slave=" << static_cast<int>(resp[0])
                  << " byteCount=" << static_cast<int>(resp[2]) << " regs=[";
        for (size_t i = 0; i < regs.size(); ++i) {
            std::cout << static_cast<int16_t>(regs[i]);
            if (i + 1 != regs.size()) std::cout << ", ";
        }
        std::cout << "] frame=[" << hexDump(resp) << "]\n";
        return regs;
    }

    void writeSingleRegister(uint8_t slave, uint16_t reg, uint16_t value, const char* tag = nullptr) {
        std::vector<uint8_t> req = {
            slave, FC_WRITE_SINGLE_REG,
            static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg),
            static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)
        };
        appendCrc(req);

        if (tag && *tag) std::cout << '[' << tag << "] ";
        std::cout << "TX(0x06): slave=" << static_cast<int>(slave)
                  << " reg=0x" << std::hex << std::setw(4) << std::setfill('0') << reg
                  << std::dec << " value=" << static_cast<int16_t>(value)
                  << " frame=[" << hexDump(req) << "]\n";

        port_.flush();
        port_.writeAll(req);

        auto resp = port_.readSome(5, 100);
        handleExceptionIfNeeded(slave, FC_WRITE_SINGLE_REG, resp);
        if (resp.size() != 8) {
            throw std::runtime_error("写单寄存器响应长度异常: " + hexDump(resp));
        }
        verifyCrc(resp);
    }

    void writeMultipleRegisters(uint8_t slave, uint16_t reg, const std::vector<uint16_t>& values,
                                const char* tag = nullptr) {
        const uint16_t count = static_cast<uint16_t>(values.size());
        std::vector<uint8_t> req = {
            slave, FC_WRITE_MULTI_REGS,
            static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg),
            static_cast<uint8_t>(count >> 8), static_cast<uint8_t>(count),
            static_cast<uint8_t>(count * 2)
        };
        for (uint16_t value : values) {
            req.push_back(static_cast<uint8_t>(value >> 8));
            req.push_back(static_cast<uint8_t>(value));
        }
        appendCrc(req);

        if (tag && *tag) std::cout << '[' << tag << "] ";
        std::cout << "TX(0x10): slave=" << static_cast<int>(slave)
                  << " startReg=0x" << std::hex << std::setw(4) << std::setfill('0') << reg
                  << std::dec << " count=" << count << " frame=[" << hexDump(req) << "]\n";

        port_.flush();
        port_.writeAll(req);

        auto resp = port_.readSome(5, 100);
        handleExceptionIfNeeded(slave, FC_WRITE_MULTI_REGS, resp);
        if (resp.size() != 8) {
            throw std::runtime_error("写多寄存器响应长度异常: " + hexDump(resp));
        }
        verifyCrc(resp);
    }

private:
    static void appendCrc(std::vector<uint8_t>& frame) {
        const uint16_t crc = crc16_modbus(frame.data(), frame.size());
        frame.push_back(static_cast<uint8_t>(crc & 0xFF));
        frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    }

    static void verifyCrc(const std::vector<uint8_t>& frame) {
        if (frame.size() < 3) throw std::runtime_error("返回帧过短");
        const uint16_t calc = crc16_modbus(frame.data(), frame.size() - 2);
        const uint16_t recv = static_cast<uint16_t>(frame[frame.size() - 2]) |
                              static_cast<uint16_t>(frame[frame.size() - 1] << 8);
        if (calc != recv) throw std::runtime_error("CRC 校验失败: " + hexDump(frame));
    }

    static void handleExceptionIfNeeded(uint8_t slave, uint8_t function, const std::vector<uint8_t>& resp) {
        if (resp.size() < 5 || resp[0] != slave || resp[1] != static_cast<uint8_t>(function | 0x80)) return;
        verifyCrc(resp);
        throw std::runtime_error("Modbus 异常响应: 0x" + std::to_string(resp[2]));
    }

    SerialPort& port_;
};

class DiffDriveController {
public:
    // 控制器只负责“左右轮命令收发”，不关心串口底层细节
    DiffDriveController(ModbusRtuMaster& mb, uint8_t leftSlaveId, uint8_t rightSlaveId)
        : mb_(mb), leftSlaveId_(leftSlaveId), rightSlaveId_(rightSlaveId) {}

    void sendWheelCommands(const WheelSpeedI16& cmd) {
        if (USE_SINGLE_REGISTER_WRITE) {
            mb_.writeSingleRegister(leftSlaveId_, REG_LEFT_CMD, static_cast<uint16_t>(cmd.left), "LEFT");
            mb_.writeSingleRegister(rightSlaveId_, REG_RIGHT_CMD, static_cast<uint16_t>(cmd.right), "RIGHT");
        } else {
            mb_.writeMultipleRegisters(leftSlaveId_, REG_LEFT_CMD, {static_cast<uint16_t>(cmd.left)}, "LEFT");
            mb_.writeMultipleRegisters(rightSlaveId_, REG_RIGHT_CMD, {static_cast<uint16_t>(cmd.right)}, "RIGHT");
        }
    }

    WheelSpeedI16 readCommandEcho() {
        auto leftRegs = mb_.readHoldingRegisters(leftSlaveId_, REG_LEFT_CMD, 1, "LEFT");
        auto rightRegs = mb_.readHoldingRegisters(rightSlaveId_, REG_RIGHT_CMD, 1, "RIGHT");
        return {static_cast<int16_t>(leftRegs[0]), static_cast<int16_t>(rightRegs[0])};
    }

    std::optional<WheelSpeedI16> readActualFeedback() {
        if (REG_LEFT_ACT == 0xFFFF || REG_RIGHT_ACT == 0xFFFF) return std::nullopt;
        auto leftRegs = mb_.readHoldingRegisters(leftSlaveId_, REG_LEFT_ACT, 1, "LEFT");
        auto rightRegs = mb_.readHoldingRegisters(rightSlaveId_, REG_RIGHT_ACT, 1, "RIGHT");
        return WheelSpeedI16{static_cast<int16_t>(leftRegs[0]), static_cast<int16_t>(rightRegs[0])};
    }

private:
    ModbusRtuMaster& mb_;
    uint8_t leftSlaveId_;
    uint8_t rightSlaveId_;
};

// 1) 输入限幅保护
static void applyInputLimit(double& v, double& w) {
    if (v > MAX_INPUT_V) v = MAX_INPUT_V;
    if (v < -MAX_INPUT_V) v = -MAX_INPUT_V;
    if (w > MAX_INPUT_W) w = MAX_INPUT_W;
    if (w < -MAX_INPUT_W) w = -MAX_INPUT_W;
}

// 2) 斜坡限制保护
static WheelSpeedI16 applyRampLimit(const WheelSpeedI16& target, const WheelSpeedI16& last) {
    auto limitOne = [](int16_t targetValue, int16_t lastValue) {
        const int delta = static_cast<int>(targetValue) - static_cast<int>(lastValue);
        if (delta > MAX_CMD_STEP_RPM) return static_cast<int16_t>(lastValue + MAX_CMD_STEP_RPM);
        if (delta < -MAX_CMD_STEP_RPM) return static_cast<int16_t>(lastValue - MAX_CMD_STEP_RPM);
        return targetValue;
    };
    return {limitOne(target.left, last.left), limitOne(target.right, last.right)};
}

// 3) 回显偏差保护
static bool checkEchoDeviation(const WheelSpeedI16& cmd, const WheelSpeedI16& echo) {
    return std::abs(int(cmd.left) - int(echo.left)) <= MAX_ECHO_DEVIATION &&
           std::abs(int(cmd.right) - int(echo.right)) <= MAX_ECHO_DEVIATION;
}

static void printEchoDeviationWarning(const WheelSpeedI16& cmd, const WheelSpeedI16& echo) {
    std::cout << "回显偏差告警: leftDiff=" << std::abs(int(cmd.left) - int(echo.left))
              << " rightDiff=" << std::abs(int(cmd.right) - int(echo.right))
              << " 阈值=" << MAX_ECHO_DEVIATION << "\n";
}

// 4) 启动/退出零输出保护
static void zeroOutput(DiffDriveController& controller) { controller.sendWheelCommands({0, 0}); }
static void startupZeroOutput(DiffDriveController& controller) { std::cout << "启动保护: 上电后先输出 0 速度\n"; zeroOutput(controller); }
static void shutdownZeroOutput(DiffDriveController& controller) { std::cout << "退出保护: 连续输出两次 0 速度\n"; zeroOutput(controller); Sleep(30); zeroOutput(controller); }

// 5) 异常自动停车
static void autoStopOnException(DiffDriveController& controller, const std::string& reason) {
    std::cout << "异常自动停车: " << reason << "\n";
    try { shutdownZeroOutput(controller); }
    catch (const std::exception& e) { std::cout << "自动停车失败: " << e.what() << "\n"; }
}

// 6) 连续通信失败保护
static bool protectCommFailure(int& count, DiffDriveController& controller, const std::string& reason) {
    ++count;
    std::cout << "通信失败次数=" << count << "/" << MAX_COMM_FAILURE_COUNT << " 原因: " << reason << "\n";
    if (count >= MAX_COMM_FAILURE_COUNT) {
        autoStopOnException(controller, "连续通信失败保护触发");
        return true;
    }
    return false;
}

static bool protectEchoFailure(int& count, DiffDriveController& controller, const WheelSpeedI16& cmd, const WheelSpeedI16& echo) {
    if (checkEchoDeviation(cmd, echo)) { count = 0; return false; }
    ++count;
    printEchoDeviationWarning(cmd, echo);
    std::cout << "回显偏差异常次数=" << count << "/" << MAX_ECHO_FAILURE_COUNT << "\n";
    if (count >= MAX_ECHO_FAILURE_COUNT) {
        autoStopOnException(controller, "连续回显偏差保护触发");
        return true;
    }
    return false;
}

int main() {
    // 主流程：初始化串口 -> 启动零输出 -> 循环读入 v/w -> 做保护处理 -> 下发命令 -> 读回显 -> 退出再零输出
    try {
        SetConsoleOutputCP(CP_UTF8);
        SetConsoleCP(CP_UTF8);

        const std::string portName = "COM5";
        const uint8_t leftSlaveId = 1;
        const uint8_t rightSlaveId = 1;
        const DWORD baudRate = CBR_19200;

        RobotParams robot;
        SerialPort serial;
        serial.openPort(portName, baudRate);
        ModbusRtuMaster mb(serial);
        DiffDriveController controller(mb, leftSlaveId, rightSlaveId);

        WheelSpeedI16 lastCmd{0, 0};
        int commFailureCount = 0;
        int echoFailureCount = 0;

        startupZeroOutput(controller);

        std::cout << "标准 Modbus RTU 双从站双驱动器差速底盘控制程序（最小必要保护版 / 现场保守参数）\n";
        std::cout << "串口=" << portName << " leftSlaveId=" << int(leftSlaveId) << " rightSlaveId=" << int(rightSlaveId) << "\n";
        std::cout << "写寄存器模式: " << (USE_SINGLE_REGISTER_WRITE ? "0x06 单寄存器写" : "0x10 多寄存器写") << "\n";
        std::cout << "左驱动目标速度寄存器: 0x4010\n右驱动目标速度寄存器: 0x4011\n";
        std::cout << "输入限幅: |v|<=" << MAX_INPUT_V << " |w|<=" << MAX_INPUT_W << "\n";
        std::cout << "斜坡限制: 每次循环最大变化=" << MAX_CMD_STEP_RPM << " rpm\n";
        std::cout << (REG_LEFT_ACT == 0xFFFF ? "实际速度反馈寄存器: 尚未填写，当前只能读命令回显\n" : "实际速度反馈寄存器已启用\n");
        std::cout << "输入: v w，例如 0.20 0.00；输入 q 退出\n\n";

        while (true) {
            std::cout << "> ";
            std::string line;
            std::getline(std::cin, line);
            if (!std::cin.good()) break;
            if (line == "q" || line == "Q") break;

            double v = 0.0, w = 0.0;
            std::istringstream iss(line);
            if (!(iss >> v >> w)) {
                std::cout << "输入格式错误\n";
                continue;
            }

            applyInputLimit(v, w);

            try {
                const WheelSpeedRpm rpm = calcWheelSpeedRpm(v, w, robot);
                const WheelSpeedI16 rawCmd = toI16(rpm, MAX_ABS_WHEEL_RPM);
                const WheelSpeedI16 cmd = applyRampLimit(rawCmd, lastCmd);

                std::cout << "rpm(left/right)=" << rpm.left << "/" << rpm.right
                          << " rawCmd(left/right)=" << rawCmd.left << "/" << rawCmd.right
                          << " rampCmd(left/right)=" << cmd.left << "/" << cmd.right << "\n";

                controller.sendWheelCommands(cmd);
                lastCmd = cmd;
                commFailureCount = 0;

                const WheelSpeedI16 echo = controller.readCommandEcho();
                if (protectEchoFailure(echoFailureCount, controller, cmd, echo)) return 1;

                if (auto fb = controller.readActualFeedback(); fb.has_value()) {
                    std::cout << "LEFT  cmd=" << cmd.left << " echo=" << echo.left << " actual=" << fb->left << "\n";
                    std::cout << "RIGHT cmd=" << cmd.right << " echo=" << echo.right << " actual=" << fb->right << "\n";
                } else {
                    std::cout << "LEFT  cmd=" << cmd.left << " echo=" << echo.left << " actual=N/A\n";
                    std::cout << "RIGHT cmd=" << cmd.right << " echo=" << echo.right << " actual=N/A\n";
                }
            } catch (const std::exception& e) {
                if (protectCommFailure(commFailureCount, controller, e.what())) return 1;
            }
        }

        shutdownZeroOutput(controller);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << "\n";
        return 1;
    }
}
