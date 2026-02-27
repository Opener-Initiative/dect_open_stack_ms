# DECT NR+ Ping Sample

This sample demonstrates the DECT NR+ stack with IPv6 connectivity, allowing ping tests between devices.

## Supported Targets

### Hardware: nRF9161 DK
Build for real hardware with DECT modem:
```bash
west build -p -b nrf9161dk/nrf9161/ns
west flash
```

### Simulation: native_sim
Build for simulation with mock PHY (no modem library):
```bash
west build -p -b native_sim/native/64
./build/zephyr/zephyr.exe
```

## Configuration

### Hardware Build (nRF9161)
Uses `prj.conf` with:
- nRF Modem Library with DECT PHY firmware
- Hardware DECT radio driver
- Security with NVS storage
- Partition Manager for flash

### Simulation Build (native_sim)
Automatically applies `boards/native_sim_64.conf` overlay with:
- Mock PHY (software simulation)
- No modem library
- No security (disabled)
- Test random generator for entropy

## Usage

### Association
The device will automatically attempt to associate:
- **FT mode**: Starts beaconing on configured channel
- **PT mode**: Scans for beacons and associates

### Network Commands
Once associated, use the network shell:
```shell
# View network interfaces
uart:~$ net iface

# Configure IPv6 address (if not auto-configured)
uart:~$ net ipv6 add <iface_id> 2001:db8:cafe::1

# Ping another device
uart:~$ net ping 2001:db8:cafe::2
```

### DECT Shell Commands
Use the `dect` shell for diagnostics:
```shell
# View MAC state
uart:~$ dect context

# View statistics
uart:~$ dect stats

# Send test data
uart:~$ dect send_data "test"
```

## Configuration Options

Edit `prj.conf` or use overlay files to configure:

```kconfig
# Set device role (choose one)
CONFIG_DECT_MAC_ROLE_IS_FT=y    # Fixed Terminal
CONFIG_DECT_MAC_ROLE_IS_PT=y    # Portable Terminal

# Set DECT channel (1880-1900 MHz)
CONFIG_DECT_MAC_DEFAULT_CARRIER=1897344  # kHz

# Enable shell commands
CONFIG_SHELL=y
CONFIG_DECT_MAC_SHELL_ENABLE=y
```

## Troubleshooting

### Build Errors

**"NRF_MODEM_LIB was assigned 'y' but got 'n'"**
- This happens when building for non-nRF91 targets
- Solution: Use `native_sim` board with the automatic overlay

**"undefined symbol DECT_NRPLUS_RADIO"**
- The hardware driver Kconfig is missing
- Solution: Ensure the dect_nrplus library module is loaded

### Runtime Issues

**"Failed to initialize DECT NR+ stack"**
- Check that the modem is active: `dmesg | grep modem`
- Verify DECT PHY firmware is loaded

**"Device is not in ASSOCIATED state"**
- For PT: Check if FT is beaconing on the same channel
- For FT: Wait for PT to send RACH
- Use `dect stats` to monitor association attempts
