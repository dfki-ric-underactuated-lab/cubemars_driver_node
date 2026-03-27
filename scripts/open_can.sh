#!/bin/bash
INTERFACES=("can0")
BITRATE=1000000
TXQUEUELEN=1000

# Function to configure CAN interface
configure_can_interface() {
    local interface=$1

    echo "Configuring $interface..."
    
    # Bring down the interface if it's up
    ip link show $interface &> /dev/null
    if [ $? -eq 0 ]; then
    sudo ip link set $interface down
    echo "$interface is down."
    else
    echo "$interface does not exist."
    return
    fi

    # Set bitrate and txqueuelen
    sudo ip link set $interface type can bitrate $BITRATE
    sudo ip link set $interface txqueuelen $TXQUEUELEN

    # Bring the interface up
    sudo ip link set $interface up
    echo "$interface is up with bitrate $BITRATE and txqueuelen $TXQUEUELEN."
}

sudo cpupower frequency-set -g performance
# Main script execution
for iface in "${INTERFACES[@]}"; do
    configure_can_interface $iface
done

# Verify configuration
echo "Verifying configuration..."
for iface in "${INTERFACES[@]}"; do
    ip -details link show $iface
done
