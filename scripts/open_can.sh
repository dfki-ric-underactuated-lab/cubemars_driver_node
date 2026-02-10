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
    ip link set $interface down
    echo "$interface is down."
    else
    echo "$interface does not exist."
    return
    fi

    # Set bitrate and txqueuelen
    ip link set $interface type can bitrate $BITRATE
    ip link set $interface txqueuelen $TXQUEUELEN

    # Bring the interface up
    ip link set $interface up
    echo "$interface is up with bitrate $BITRATE and txqueuelen $TXQUEUELEN."
}

# Main script execution
for iface in "${INTERFACES[@]}"; do
    configure_can_interface $iface
done

# Verify configuration
echo "Verifying configuration..."
for iface in "${INTERFACES[@]}"; do
    ip -details link show $iface
done
