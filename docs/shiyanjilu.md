orangepi@orangepi5plus:~/ROV_ctrl_2026/Underwater-robot-navigation/nav_core$ cmake --build build --target uwnav_navd
Scanning dependencies of target nav_core
[  4%] Building CXX object CMakeFiles/nav_core.dir/src/nav_core/drivers/imu_driver_wit.cpp.o
[  8%] Linking CXX static library libnav_core.a
[ 88%] Built target nav_core
[ 92%] Linking CXX executable bin/uwnav_navd
[100%] Built target uwnav_navd
orangepi@orangepi5plus:~/ROV_ctrl_2026/Underwater-robot-navigation/nav_core$ ./build/bin/uwnav_navd --config ~/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/nav_daemon.yaml --eskf-config ~/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/eskf.yaml
[nav_daemon] starting with config=/home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/nav_daemon.yaml eskf_config=/home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/eskf.yaml
[nav_daemon_config][INFO] loading nav_daemon config from: /home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/nav_daemon.yaml
[nav_daemon_config][WARN] failed to parse key 'slave_addr': yaml-cpp: error at line 46, column 17: bad conversion
[nav_daemon_config][INFO] nav_daemon config loaded successfully.
[eskf_config_yaml][INFO] loading EskfConfig from: /home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/eskf.yaml
[eskf_config_yaml][INFO] EskfConfig loaded successfully.
[nav_daemon] NavStatePublisher initialized (shm=/rov_nav_state_v1)
[BinLogger] mkdir('/home/wys') failed: Permission denied
[BinLogger] ensureDirForFile('/home/wys/data/nav/2026-03-30/nav/nav.bin') failed
[nav_daemon] WARNING: BinLogger::open('/home/wys/data/nav/2026-03-30/nav/nav.bin') failed, disable logging
[nav_daemon] WARNING: sample BinLogger init failed or disabled (不会写处理后的 DVL 样本日志，但不影响导航本身)
[BinLogger] mkdir('/home/wys') failed: Permission denied
[BinLogger] ensureDirForFile('/home/wys/data/nav/2026-03-30/nav/nav_timing.bin') failed
[nav_daemon] WARNING: BinLogger::open('/home/wys/data/nav/2026-03-30/nav/nav_timing.bin') failed, disable logging
[nav_daemon] WARNING: timing BinLogger init failed or disabled (不会写时间语义追踪日志，但不影响导航本身)
[BinLogger] mkdir('/home/wys') failed: Permission denied
[BinLogger] ensureDirForFile('/home/wys/data/nav/2026-03-30/nav/nav_state.bin') failed
[nav_daemon] WARNING: BinLogger::open('/home/wys/data/nav/2026-03-30/nav/nav_state.bin') failed, disable logging
[nav_daemon] WARNING: nav_state BinLogger init failed or disabled (不会写导航状态日志，但不影响导航本身)
[nav_daemon] WARNING: nav_events.csv init failed or disabled (结构化低频事件日志不可用，但不影响导航主链)
[nav_daemon] READY: device binders armed, ESKF online, loop=50.00 Hz
[nav_daemon] main loop started (50.00 Hz)
[nav_daemon] READY: online navigation is running (IMU=ENABLED, DVL=DISABLED, logging=OFF)
[IMU] serial opened: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400
[IMU] started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400 (Modbus, addr=0x50)
[nav_daemon] IMU driver started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @ 230400
[nav_daemon] IMU state=ONLINE path=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 reason=driver online
[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline
[nav_daemon] NAV t=7101.7s E=0.000 N=0.000 U=0.000 depth=-0.00 vE=0.000 vN=0.000 vU=0.000 yaw=0.000 valid=0 stale=0 degraded=0 state=0 health=0 fault=10 age_ms=4294967295 sensor_mask=0x0000 flags=0x0040
[nav_daemon] IMU opened but no parseable frame arrived within 1000 ms (path=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0, baud=230400, addr=0x50)
[nav_daemon] HINT: 串口已打开，但 IMU 没有返回可解析的 Modbus 数据；请检查供电、RS485 A/B、波特率、slave_addr 和 USB-RS485 转接器
[IMU] serial closed
[IMU] threadFunc exit
[IMU] stopped
[nav_daemon] IMU state=RECONNECTING path=<none> reason=no IMU frame after connect
[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline
[IMU] serial opened: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400
[IMU] started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400 (Modbus, addr=0x50)
[nav_daemon] IMU driver started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @ 230400
[nav_daemon] IMU state=ONLINE path=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 reason=driver online
[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline
[nav_daemon] IMU opened but no parseable frame arrived within 1019 ms (path=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0, baud=230400, addr=0x50)
[nav_daemon] HINT: 串口已打开，但 IMU 没有返回可解析的 Modbus 数据；请检查供电、RS485 A/B、波特率、slave_addr 和 USB-RS485 转接器
[IMU] serial closed
[IMU] threadFunc exit
[IMU] stopped
[nav_daemon] IMU state=RECONNECTING path=<none> reason=no IMU frame after connect
[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline
[IMU] serial opened: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400
[IMU] started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400 (Modbus, addr=0x50)
[nav_daemon] IMU driver started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @ 230400
[nav_daemon] IMU state=ONLINE path=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 reason=driver online
[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline
[nav_daemon] IMU opened but no parseable frame arrived within 1020 ms (path=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0, baud=230400, addr=0x50)
[nav_daemon] HINT: 串口已打开，但 IMU 没有返回可解析的 Modbus 数据；请检查供电、RS485 A/B、波特率、slave_addr 和 USB-RS485 转接器
[IMU] serial closed
[IMU] threadFunc exit
[IMU] stopped
[nav_daemon] IMU state=RECONNECTING path=<none> reason=no IMU frame after connect
[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline
[IMU] serial opened: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400
[IMU] started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400 (Modbus, addr=0x50)
[nav_daemon] IMU driver started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @ 230400
[nav_daemon] IMU state=ONLINE path=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 reason=driver online
[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline
[nav_daemon] IMU opened but no parseable frame arrived within 1019 ms (path=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0, baud=230400, addr=0x50)
[nav_daemon] HINT: 串口已打开，但 IMU 没有返回可解析的 Modbus 数据；请检查供电、RS485 A/B、波特率、slave_addr 和 USB-RS485 转接器
[IMU] serial closed
[IMU] threadFunc exit
[IMU] stopped
[nav_daemon] IMU state=RECONNECTING path=<none> reason=no IMU frame after connect
[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline
[IMU] serial opened: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400
[IMU] started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @230400 (Modbus, addr=0x50)
[nav_daemon] IMU driver started on /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @ 230400
[nav_daemon] IMU state=ONLINE path=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 reason=driver online
[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline
^C[nav_daemon] main loop exiting (stop requested)
[IMU] serial closed
[IMU] threadFunc exit
[IMU] stopped
[nav_daemon] exited with code 0


(py311) orangepi@orangepi5plus:~/ROV_ctrl_2026/Underwater-robot-navigation/nav_core$ sudo ./build/bin/uwnav_navd --config /home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/nav_daemon.yaml --eskf-config /home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/eskf.yaml
[nav_daemon] starting with config=/home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/nav_daemon.yaml eskf_config=/home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/eskf.yaml
[nav_daemon_config][INFO] loading nav_daemon config from: /home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/nav_daemon.yaml
[nav_daemon_config][INFO] nav_daemon config loaded successfully.
[eskf_config_yaml][INFO] loading EskfConfig from: /home/orangepi/ROV_ctrl_2026/Underwater-robot-navigation/nav_core/config/eskf.yaml
[eskf_config_yaml][INFO] EskfConfig loaded successfully.
[nav_daemon] NavStatePublisher initialized (shm=/rov_nav_state_v1)
[nav_daemon] bin_logger logging to /home/wys/data/nav/2026-03-31/nav/nav.bin
[nav_daemon] bin_logger logging to /home/wys/data/nav/2026-03-31/nav/nav_timing.bin
[nav_daemon] bin_logger logging to /home/wys/data/nav/2026-03-31/nav/nav_state.bin
[nav_daemon] READY: device binders armed, ESKF online, loop=50.00 Hz
[nav_daemon] main loop started (50.00 Hz)
[nav_daemon] READY: online navigation is running (IMU=ENABLED, DVL=ENABLED, logging=ON)
[nav_daemon] IMU state=MISMATCH path=<none> reason=IMU auto probe rejected 3 serial candidate(s): /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 passive=unknown active=unknown; /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 passive=unknown active=unknown; /dev/ttyUSB0 passive=volt32_ascii(CH0:0000     0.000V)
[DVL] serial opened: /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 @115200
[DVL] started on /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 @115200
[nav_daemon] DVL driver started on /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 @ 115200
[nav_daemon] DVL state=ONLINE path=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 reason=driver online
[nav_daemon] DVL connectivity changed; clear DVL freshness cache
[nav_daemon] health audit changed: health=3 cause=transport_timing summary=imu_alive=0 dvl_alive=0 imu_age_s=inf dvl_age_s=inf imu_timing(stale=0,ooo=0) dvl_timing(stale=0,ooo=0) action=stop
[nav_daemon] NAV t=1077.2s E=0.000 N=0.000 U=0.000 depth=-0.00 vE=0.000 vN=0.000 vU=0.000 yaw=0.000 valid=0 stale=0 degraded=0 state=0 health=0 fault=11 age_ms=4294967295 sensor_mask=0x0000 flags=0x0180
[nav_daemon] IMU state=MISMATCH path=<none> reason=IMU auto probe rejected 3 serial candidate(s): /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 passive=unknown active=unknown; /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 passive=unknown active=unknown; /dev/ttyUSB0 passive=volt32_ascii(CH0:0000     0.000V)
[nav_daemon] IMU state=MISMATCH path=<none> reason=IMU auto probe rejected 3 serial candidate(s): /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 passive=unknown active=unknown; /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 passive=unknown active=unknown; /dev/ttyUSB0 passive=unknown active=other_ascii(x~ xxxx  x  xxxf        x~ xxxx  x  xxx         ...)
[DVL] serial closed
[DVL] threadFunc exit. stats: lines=0, parsed_ok=0, parsed_fail=0
[DVL] stopped
[nav_daemon] DVL state=RECONNECTING path=<none> reason=no DVL frame after connect
[nav_daemon] DVL connectivity changed; clear DVL freshness cache
[nav_daemon] IMU state=MISMATCH path=<none> reason=IMU auto probe rejected 3 serial candidate(s): /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 passive=unknown active=unknown; /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 passive=unknown active=unknown; /dev/ttyUSB0 passive=volt32_ascii(CH0:0000     0.000V)
[DVL] serial opened: /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 @115200
[DVL] started on /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 @115200
[nav_daemon] DVL driver started on /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 @ 115200
[nav_daemon] DVL state=ONLINE path=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 reason=driver online
[nav_daemon] DVL connectivity changed; clear DVL freshness cache
[nav_daemon] IMU state=MISMATCH path=<none> reason=IMU auto probe rejected 3 serial candidate(s): /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 passive=unknown active=unknown; /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 passive=unknown active=unknown; /dev/ttyUSB0 passive=unknown active=other_ascii(x~ xxxx  x  xxxf        x~ xxxx  x  xxx         ...)
[nav_daemon] IMU state=MISMATCH path=<none> reason=IMU auto probe rejected 3 serial candidate(s): /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 passive=unknown active=unknown; /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 passive=unknown active=unknown; /dev/ttyUSB0 passive=volt32_ascii(CH0:0000     0.000V)
[DVL] serial closed
[DVL] threadFunc exit. stats: lines=0, parsed_ok=0, parsed_fail=0
[DVL] stopped
[nav_daemon] DVL state=RECONNECTING path=<none> reason=no DVL frame after connect
[nav_daemon] DVL connectivity changed; clear DVL freshness cache
[nav_daemon] IMU state=MISMATCH path=<none> reason=IMU auto probe rejected 3 serial candidate(s): /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 passive=unknown active=unknown; /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 passive=unknown active=unknown; /dev/ttyUSB0 passive=volt32_ascii(CH3:0000     0.000A)
[DVL] serial opened: /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 @115200
[DVL] started on /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 @115200
[nav_daemon] DVL driver started on /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 @ 115200
[nav_daemon] DVL state=ONLINE path=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 reason=driver online
[nav_daemon] DVL connectivity changed; clear DVL freshness cache
[nav_daemon] IMU state=MISMATCH path=<none> reason=IMU auto probe rejected 3 serial candidate(s): /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 passive=unknown active=unknown; /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 passive=unknown active=unknown; /dev/ttyUSB0 passive=volt32_ascii(CH0:0000     0.000V)
[nav_daemon] IMU state=MISMATCH path=<none> reason=IMU auto probe rejected 3 serial candidate(s): /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 passive=unknown active=unknown; /dev/serial/by-id/usb-1a86_USB_Single_Serial_5959122002-if00 passive=unknown active=unknown; /dev/ttyUSB0 passive=volt32_ascii(CH0:0000     0.000V)
[DVL] serial closed
^C[DVL] threadFunc exit. stats: lines=0, parsed_ok=0, parsed_fail=0
[DVL] stopped
[nav_daemon] DVL state=RECONNECTING path=<none> reason=no DVL frame after connect
[nav_daemon] DVL connectivity changed; clear DVL freshness cache
[nav_daemon] main loop exiting (stop requested)
[nav_daemon] exited with code 0