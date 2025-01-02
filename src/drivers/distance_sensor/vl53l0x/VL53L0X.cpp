#include "VL53L0X.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

VL53L0X *VL53L0X::_instance = nullptr;
#define VL53L0X_SAMPLE_RATE 50000 // 50ms

VL53L0X::VL53L0X() : px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}

VL53L0X::~VL53L0X()
{
    ScheduleClear();
    if (_uart_fd >= 0) {
        ::close(_uart_fd); // Release UART resource
    }
}

int VL53L0X::setup_uart()
{
    const char *uart_path = "/dev/ttyS3"; // SEUK:: UART 포트 경로
    _uart_fd = ::open(uart_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (_uart_fd < 0) {
        PX4_ERR("Failed to open UART port: %s", uart_path);
        return PX4_ERROR;
    }

    struct termios uart_config;
    if (tcgetattr(_uart_fd, &uart_config) < 0) {
        PX4_ERR("Failed to get UART configuration");
        ::close(_uart_fd);
        return PX4_ERROR;
    }

    // 보드레이트 설정 (57600bps)
    cfsetispeed(&uart_config, B57600);
    cfsetospeed(&uart_config, B57600);

    // 제어 플래그 설정
    uart_config.c_cflag |= (CLOCAL | CREAD); // 로컬 연결, 데이터 수신 활성화
    uart_config.c_cflag &= ~CSIZE;          // 데이터 크기 초기화
    uart_config.c_cflag |= CS8;             // 8비트 데이터
    uart_config.c_cflag &= ~PARENB;         // 패리티 비트 사용 안 함
    uart_config.c_cflag &= ~CSTOPB;         // 1비트 스톱 비트
    uart_config.c_cflag &= ~CRTSCTS;        // 하드웨어 흐름 제어 비활성화

    // 입력 플래그 설정
    uart_config.c_iflag &= ~(IXON | IXOFF | IXANY); // 소프트웨어 흐름 제어 비활성화
    uart_config.c_iflag &= ~(ICRNL | INLCR);        // 캐리지 리턴 변환 비활성화

    // 출력 플래그 설정
    uart_config.c_oflag &= ~OPOST;         // 출력 처리 비활성화

    // 로우 모드 설정
    uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 로우 모드 설정

    // 최소 읽기 및 타이머 설정
    uart_config.c_cc[VMIN] = 0;   // 최소 읽기 바이트 수 (0 = 비블로킹)
    uart_config.c_cc[VTIME] = 10; // 읽기 타임아웃 (단위: 100ms)

    // UART 구성 적용
    if (tcsetattr(_uart_fd, TCSANOW, &uart_config) < 0) {
        PX4_ERR("Failed to set UART configuration");
        ::close(_uart_fd);
        return PX4_ERROR;
    }

    PX4_INFO("UART configured successfully on %s", uart_path);
    return PX4_OK;
}

int VL53L0X::init()
{
    if (setup_uart() != PX4_OK) {
        return PX4_ERROR;
    }
    ScheduleNow(); // Schedule the first run
    return PX4_OK;
}

void VL53L0X::Run()
{
    if (tx_pending) {
        if (rx_function()) { // 응답을 정상적으로 받았을 경우
            tx_pending = false; // 상태 초기화
        }
    } else {
        tx_function(); // 송신
        tx_pending = true; // 송신 상태로 설정
    }

    // 주기적으로 호출
    ScheduleDelayed(VL53L0X_SAMPLE_RATE);
}

void VL53L0X::tx_function()
{
    ssize_t bytes_written = ::write(_uart_fd, request_packet, sizeof(request_packet)); // 명시적으로 크기 전달

    if (bytes_written < 0) {
        PX4_WARN("Failed to transmit data");
    } else {
        PX4_INFO("Transmitted %zd bytes", bytes_written);
    }
}


bool VL53L0X::rx_function()
{
    memset(response_packet, 0, sizeof(response_packet)); // 버퍼 초기화
    ssize_t bytes_read = ::read(_uart_fd, response_packet, sizeof(response_packet));

    if (bytes_read > 0) {
        PX4_INFO("Received %zd bytes", bytes_read);

        // 패킷 유효성 검증
        if (validate_packet(response_packet, bytes_read)) {
            PX4_INFO("Valid packet received");
            return true; // 응답 처리 완료
        } else {
            PX4_WARN("Invalid packet received");
        }
    } else {
        PX4_WARN("No data received");
    }
    return false; // 응답 처리 실패
}

bool VL53L0X::validate_packet(const uint8_t *packet, ssize_t length)
{
    if (length < 4) return false; // 최소 패킷 길이 확인

    // 패킷 시작 확인
    if (packet[0] != 0xFF || packet[1] != 0xFF) return false;

    // 체크섬 확인
    uint8_t checksum = 0;
    for (ssize_t i = 2; i < length - 1; ++i) {
        checksum += packet[i];
    }
    checksum = ~checksum;

    return checksum == packet[length - 1]; // 체크섬 유효성
}

// send_tx 수정
void VL53L0X::send_tx(const char *value)
{
    // 문자열 값을 패킷으로 변환 및 저장
    memset(request_packet, 0, sizeof(request_packet)); // 기존 패킷 초기화
    size_t length = strlen(value);

    if (length > sizeof(request_packet)) {
        PX4_WARN("Value is too long for the packet");
        return;
    }

    memcpy(request_packet, value, length); // 패킷 복사
    PX4_INFO("Request packet updated: %s", value);
}

void VL53L0X::send_rx()
{
    PX4_INFO("Current response packet: %s", response_packet);
}

void VL53L0X::print_usage()
{
    PRINT_MODULE_USAGE_NAME("dynamixel", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");
}

extern "C" __EXPORT int dynamixel_main(int argc, char *argv[])
{
    if (argc < 2) {
        VL53L0X::print_usage();
        return -1;
    }

    const char *verb = argv[1];

    if (!strcmp(verb, "start")) {
        if (VL53L0X::_instance == nullptr) {
            VL53L0X::_instance = new VL53L0X();
            return VL53L0X::_instance->init();
        } else {
            PX4_WARN("Dynamixel already running");
            return -1;
        }
    }

    if (!strcmp(verb, "stop")) {
        if (VL53L0X::_instance != nullptr) {
            delete VL53L0X::_instance;
            VL53L0X::_instance = nullptr;
            PX4_INFO("Dynamixel stopped");
        } else {
            PX4_WARN("Dynamixel not running");
        }
        return 0;
    }

    if (!strcmp(verb, "status")) {
        if (VL53L0X::_instance != nullptr) {
            PX4_INFO("Dynamixel is running");
        } else {
            PX4_WARN("Dynamixel not running");
        }
        return 0;
    }

    if (!strcmp(verb, "tx")) {
        if (argc < 3) {
            PX4_WARN("Usage: Dynamixel tx <value>");
            return -1;
        }

        if (VL53L0X::_instance == nullptr) {
            PX4_WARN("Dynamixel is not running. Start it first.");
            return -1;
        }

        const char *value = argv[2];
        VL53L0X::_instance->send_tx(value);
        return 0;
    }

    if (!strcmp(verb, "rx")) {
        if (VL53L0X::_instance == nullptr) {
            PX4_WARN("Dynamixel is not running. Start it first.");
            return -1;
        }

        VL53L0X::_instance->send_rx();
        return 0;
    }

    VL53L0X::print_usage();
    return -1;
}
