#!/bin/bash
# CAN Debug Script for G4 Motor Driver
# This script provides convenient commands for debugging the motor controller via CAN

set -e

# CAN interface (change if using different interface)
CAN_INTERFACE="${CAN_INTERFACE:-slcan0}"

# CAN IDs (matching can_protocol.rs)
SPEED_CMD_ID="100"
PI_GAINS_ID="101"
ENABLE_CMD_ID="102"
STATUS_ID="200"
VOLTAGE_STATUS_ID="201"
EMERGENCY_STOP_ID="000"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper function to convert f32 to little-endian hex bytes
f32_to_hex() {
    python3 -c "import struct; print(''.join(f'{b:02X}' for b in struct.pack('<f', $1)))"
}

# Helper function to decode f32 from hex bytes
hex_to_f32() {
    python3 -c "import struct; print(struct.unpack('<f', bytes.fromhex('$1'))[0])"
}

# Setup and check CAN interface
setup_interface() {
    # Check if interface exists
    if ! ip link show "$CAN_INTERFACE" &> /dev/null; then
        echo -e "${YELLOW}CAN interface $CAN_INTERFACE not found, attempting to set up...${NC}"

        # Try to setup slcan interface
        if [[ "$CAN_INTERFACE" == slcan* ]]; then
            # Look for USB serial device (common for slcan adapters)
            local serial_device=""
            for dev in /dev/ttyACM* /dev/ttyUSB*; do
                if [ -e "$dev" ]; then
                    serial_device="$dev"
                    break
                fi
            done

            if [ -z "$serial_device" ]; then
                echo -e "${RED}Error: No USB serial device found (/dev/ttyACM* or /dev/ttyUSB*)${NC}"
                echo "Please connect your CAN adapter and try again"
                exit 1
            fi

            echo -e "${BLUE}Found serial device: $serial_device${NC}"
            echo -e "${BLUE}Setting up slcan interface...${NC}"

            # Setup slcan (S6 = 250kbps)
            sudo slcand -o -c -s6 "$serial_device" "$CAN_INTERFACE" || {
                echo -e "${RED}Error: Failed to setup slcan interface${NC}"
                echo "Try: sudo slcand -o -c -s6 $serial_device $CAN_INTERFACE"
                exit 1
            }

            sleep 1
            echo -e "${GREEN}slcan interface created successfully${NC}"
        else
            # For hardware CAN interfaces (can0, can1, etc.)
            echo -e "${RED}Error: CAN interface $CAN_INTERFACE not found${NC}"
            echo "For hardware CAN interfaces, please ensure the device tree or kernel module is loaded"
            exit 1
        fi
    fi

    # Check if interface is UP
    if ! ip link show "$CAN_INTERFACE" | grep -q "UP"; then
        echo -e "${YELLOW}CAN interface $CAN_INTERFACE is DOWN, bringing it up...${NC}"

        # Try to bring up the interface
        sudo ip link set "$CAN_INTERFACE" up || {
            echo -e "${RED}Error: Failed to bring up $CAN_INTERFACE${NC}"
            echo "Try manually: sudo ip link set $CAN_INTERFACE up"
            exit 1
        }

        sleep 0.5
        echo -e "${GREEN}CAN interface $CAN_INTERFACE is now UP${NC}"
    else
        echo -e "${GREEN}CAN interface $CAN_INTERFACE is ready${NC}"
    fi
}

# Send speed command
send_speed() {
    local speed=$1
    if [ -z "$speed" ]; then
        echo "Usage: $0 speed <RPM>"
        echo "Example: $0 speed 1000"
        exit 1
    fi

    local hex_data=$(f32_to_hex "$speed")
    echo -e "${GREEN}Sending speed command: ${speed} RPM${NC}"
    echo "CAN frame: $CAN_INTERFACE  $SPEED_CMD_ID#$hex_data"
    cansend "$CAN_INTERFACE" "$SPEED_CMD_ID#$hex_data"
}

# Send PI gains
send_pi_gains() {
    local kp=$1
    local ki=$2
    if [ -z "$kp" ] || [ -z "$ki" ]; then
        echo "Usage: $0 pi <Kp> <Ki>"
        echo "Example: $0 pi 0.1 0.01"
        exit 1
    fi

    local kp_hex=$(f32_to_hex "$kp")
    local ki_hex=$(f32_to_hex "$ki")
    local hex_data="${kp_hex}${ki_hex}"

    echo -e "${GREEN}Sending PI gains: Kp=$kp, Ki=$ki${NC}"
    echo "CAN frame: $CAN_INTERFACE  $PI_GAINS_ID#$hex_data"
    cansend "$CAN_INTERFACE" "$PI_GAINS_ID#$hex_data"
}

# Enable motor
motor_enable() {
    echo -e "${GREEN}Enabling motor${NC}"
    cansend "$CAN_INTERFACE" "$ENABLE_CMD_ID#01"
}

# Disable motor
motor_disable() {
    echo -e "${YELLOW}Disabling motor${NC}"
    cansend "$CAN_INTERFACE" "$ENABLE_CMD_ID#00"
}

# Emergency stop
emergency_stop() {
    echo -e "${RED}EMERGENCY STOP!${NC}"
    cansend "$CAN_INTERFACE" "$EMERGENCY_STOP_ID#00"
}

# Monitor status messages
monitor_status() {
    echo -e "${BLUE}Monitoring motor status (ID 0x$STATUS_ID) and voltage (ID 0x$VOLTAGE_STATUS_ID)...${NC}"
    echo "Press Ctrl+C to stop"
    echo ""

    candump "$CAN_INTERFACE" | while read -r line; do
        # Parse candump output: "slcan0  200   [8]  00 00 00 00 00 00 00 00"
        if echo "$line" | grep -q " $STATUS_ID "; then
            # Extract hex data
            hex_data=$(echo "$line" | awk '{print $4$5$6$7$8$9$10$11}' | tr -d ' ')

            if [ ${#hex_data} -eq 16 ]; then
                # Extract speed (first 4 bytes) and angle (last 4 bytes)
                speed_hex=${hex_data:0:8}
                angle_hex=${hex_data:8:8}

                speed=$(hex_to_f32 "$speed_hex")
                angle=$(hex_to_f32 "$angle_hex")

                echo -e "${GREEN}[$(date +%H:%M:%S.%3N)]${NC} Speed: ${BLUE}${speed}${NC} RPM, Angle: ${BLUE}${angle}${NC} rad"
            fi
        elif echo "$line" | grep -q " $VOLTAGE_STATUS_ID "; then
            # Extract hex data for voltage status
            hex_data=$(echo "$line" | awk '{print $4$5$6$7$8$9}' | tr -d ' ')

            if [ ${#hex_data} -ge 10 ]; then
                # Extract voltage (first 4 bytes) and flags (5th byte)
                voltage_hex=${hex_data:0:8}
                flags_hex=${hex_data:8:2}

                voltage=$(hex_to_f32 "$voltage_hex")
                flags=$((16#$flags_hex))

                # Parse flags (bit 0: overvoltage, bit 1: undervoltage)
                overvoltage=$((flags & 0x01))
                undervoltage=$((flags & 0x02))

                status_str=""
                if [ $overvoltage -ne 0 ]; then
                    status_str="${RED}OVERVOLTAGE${NC}"
                elif [ $undervoltage -ne 0 ]; then
                    status_str="${RED}UNDERVOLTAGE${NC}"
                else
                    status_str="${GREEN}OK${NC}"
                fi

                echo -e "${GREEN}[$(date +%H:%M:%S.%3N)]${NC} Voltage: ${BLUE}${voltage}${NC} V, Status: $status_str"
            fi
        fi
    done
}

# Dump all CAN traffic
dump_all() {
    echo -e "${BLUE}Dumping all CAN traffic on $CAN_INTERFACE...${NC}"
    echo "Press Ctrl+C to stop"
    candump "$CAN_INTERFACE"
}

# Interactive sniffer
sniffer() {
    echo -e "${BLUE}Starting CAN sniffer on $CAN_INTERFACE...${NC}"
    echo "Press Ctrl+C to stop"
    cansniffer "$CAN_INTERFACE"
}

# Test sequence
test_sequence() {
    echo -e "${BLUE}Running test sequence...${NC}"
    echo ""

    echo "1. Set PI gains to default (Kp=0.5, Ki=0.05)"
    send_pi_gains 0.5 0.05
    sleep 1

    echo ""
    echo "2. Enable motor"
    motor_enable
    sleep 1

    echo ""
    echo "3. Ramp up speed: 0 -> 500 -> 1000 RPM"
    for speed in 0 100 200 300 400 500 600 700 800 900 1000; do
        send_speed "$speed"
        sleep 0.5
    done

    echo ""
    echo "4. Hold at 1000 RPM for 3 seconds"
    sleep 3

    echo ""
    echo "5. Ramp down: 1000 -> 500 -> 0 RPM"
    for speed in 900 800 700 600 500 400 300 200 100 0; do
        send_speed "$speed"
        sleep 0.5
    done

    echo ""
    echo "6. Disable motor"
    motor_disable

    echo ""
    echo -e "${GREEN}Test sequence completed!${NC}"
}

# Show usage
usage() {
    echo "CAN Debug Script for G4 Motor Driver"
    echo ""
    echo "Usage: $0 <command> [args...]"
    echo ""
    echo "Commands:"
    echo "  speed <RPM>         Send speed command (f32 RPM)"
    echo "  pi <Kp> <Ki>        Set PI controller gains (f32, f32)"
    echo "  enable              Enable motor"
    echo "  disable             Disable motor"
    echo "  estop               Emergency stop"
    echo "  monitor             Monitor motor status (ID 0x200) and voltage (ID 0x201)"
    echo "  dump                Dump all CAN traffic"
    echo "  sniffer             Interactive CAN sniffer"
    echo "  test                Run test sequence"
    echo ""
    echo "CAN Protocol:"
    echo "  0x100: Speed command (f32 RPM, 4 bytes)"
    echo "  0x101: PI gains (Kp: f32, Ki: f32, 8 bytes)"
    echo "  0x102: Motor enable (u8: 0=disable, 1=enable)"
    echo "  0x200: Motor status (speed: f32, angle: f32, 8 bytes)"
    echo "  0x201: Voltage status (voltage: f32, flags: u8, 5 bytes)"
    echo "  0x000: Emergency stop"
    echo ""
    echo "Examples:"
    echo "  $0 speed 1000              # Set speed to 1000 RPM"
    echo "  $0 pi 0.5 0.05             # Set Kp=0.5, Ki=0.05 (default)"
    echo "  $0 enable                  # Enable motor"
    echo "  $0 monitor                 # Monitor status messages"
    echo ""
    echo "Environment:"
    echo "  CAN_INTERFACE=$CAN_INTERFACE (can be changed with CAN_INTERFACE=can0 $0 ...)"
}

# Main
setup_interface

case "${1:-}" in
    speed)
        send_speed "$2"
        ;;
    pi)
        send_pi_gains "$2" "$3"
        ;;
    enable)
        motor_enable
        ;;
    disable)
        motor_disable
        ;;
    estop)
        emergency_stop
        ;;
    monitor)
        monitor_status
        ;;
    dump)
        dump_all
        ;;
    sniffer)
        sniffer
        ;;
    test)
        test_sequence
        ;;
    *)
        usage
        exit 1
        ;;
esac
