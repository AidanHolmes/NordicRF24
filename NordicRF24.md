# NordicRF24 class reference

## Initialisation

### set_spi(IHardwareSPI *pSPI)
Takes a reference to an SPI hardware interface. SPI needs to be configured prior to setting.
Returns false if the SPI interface is invalid

### set_gpio(IHardwareGPIO *pGPIO, uint8_t ce, uint8_t irq)
Takes a reference to a GPIO hardware interface. The pin numbers for CE and IRQ also need to be specified in BCM format. This function configures the pins using the GPIO interface provided
Returns false if the GPIO interface is invalid

### reset_rf24()
Resets the RF24 hardware and resets all class attributes.
This will need to be called after providing SPI and GPIO interfaces.
Returns false if the reset failed

### auto_update(bool update)
If update is set to true then every read and write call to registers will be pulled/pushed over SPI.
If update is set to false then separate read_ and write_ calls will be needed to sync the changes to the hardware.

## Link layer functions

### write_packet(uint8_t *packet)
Writes a packet of data and pulses the CE to send the data.
This will only work if the RF24 is is transmit mode and powered up:
*power_up(true);*
*receiver(false) ;*
packet buffer must match required length of packet data as set by:
*set_transmit_width(uint8_t width);*

### get_rx_data_size(uint8_t pipe)
This is a helper function which reads the pipe data size or the dynamic data size depending on the RF24 settings.
Returns 0 if there's an error with SPI, GPIO or dynamic data size is corrupt.

### read_payload(uint8_t *buffer, uint8_t len)
Reads from the RX buffer. len and buffer must match the RX data size read using:
*get_rx_data_size(uint8_t pipe);*
Returns false if call fails

### write_payload(uint8_t *buffer, uint8_t len)
Writes a packet of data to the TX buffer. This call will not pulse CE to send data. len and buffer can be any length up to 32 bytes, but to keep things simple it's suggested that the length matches if fixed payload sizes are used:
*set_transmit_width(uint8_t width);*

## Hardware configuration functions

### Configuration register
#### read_config()
Reads the configuration settings from the RF24 hardware into the class. All values are cached in the class
Returns false if there's an error

#### write_config()
Writes all configuration settings to the RF24 hardware from the class cache.
Returns false if there's an error

#### use_interrupt_data_ready()
Returns true if the DR interrupt is in use

#### use_interrupt_data_sent()
Returns true if the DS interrupt is in use

#### use_interrupt_max_retry()
Returns true if the RT interrupt is in use

#### is_crc_enabled()
Returns true if CRC functionality is enabled

#### is_powered_up()
Returns true if the RF24 has been powered up.

#### is_receiver()
Returns true if the RF24 device is running as a receiver. Returns false if the device is a transmitter.

#### is_2_byte_crc()
Returns true if 16bit CRC is in use. Returns false if 8bit CRC is in use

#### set_use_interrupt_data_ready(bool set)
Specify if the interrupt should be set if data is ready.
Set to true to activate the interrupt, false to deactivate

#### set_use_interrupt_data_sent(bool set)
Specify if the interrupt should be set if data has been sent
Set to true to activate the interrupt, false to deactivate

#### set_use_interrupt_max_retry(bool set)
Specify if the interrupt should be set if the max retries has been exceeded
Set to true to activate the interrupt, false to deactivate

#### crc_enabled(bool set)
Enable or disable CRC

#### set_2_byte_crc(bool set)
Set CRC to 8 or 16 bit. If set is true then it will use 16 bit CRC

#### power_up(bool set)
If set is true then then it will be powered up. If false then the RF24 hardware will be set to powered down state

#### receiver(bool set)
Set to true to act as a receiver. Use false to act as a transmitter.

### Auto acknowledgement
#### read_enaa()
Pull settings from the RF24 hardware for the auto acknowledgement register. This reads the settings for all pipes. This is a required call if auto update is not enabled.
Returns false if the read fails.

#### write_enaa()
Write all pipe ack states to the RF24 hardware. This call is required if auto update is disabled.
Returns false if the write fails.

#### is_pipe_ack(uint8_t pipe)
Check if a *pipe* has auto acknowledgement enabled.
Returns true if set and false if disabled.

#### set_pipe_ack(uint8_t pipe, bool val)
Sets a pipe auto acknowledgement state.
*pipe* defines which to set and *val* specifies if auto acknowledgement is enabled.

### Enable pipes
Pipes 0 and 1 are enabled by default. To control which are enabled and disabled use these functions.
#### read_enrxaddr()
This is a manual read function to pull settings from the device.
If auto_update() is enabled then this will be called automatically.
Returns false if call fails with hardware.

#### write_enrxaddr()
This is a manual write function to push settings to the device.
If auto_update() is enabled then this will be called automatically.
Returns false if call fails with hardware.

#### is_pipe_enabled(uint8_t pipe)
*pipe* is a valid pipe number from 0 to 5.
Returns true if *pipe* is enabled, else returns false.
Invalid pipe numbers cause the function to return false.

#### enable_pipe(uint8_t pipe, bool enabled)
*pipe* is a valid pipe number from 0 to 5. Invalid pipe numbers will cause the function to do nothing.
*enabled* defines if the pipe is enabled - true or disabled - false.

### Address width management
Addresses can be 3-5 bytes long.

#### set_address_width(uint8_t width)
Define the address width to use. The affects all pipes and TX address settings
Returns false if an error occurred or the address width is invalid

#### get_address_width()
Returns the width used for the addresses. Can be 3, 4 or 5 bytes.
Returns 0 if an error has occurred.

### Retry settings
Sets delays from 250-4000 micro seconds and the number of retries attempted.

#### set_retry(uint8_t delay_multiplier, uint8_t retry_count)
Sets both parameters at once.
*delay_multiplier* sets the delay in multiples of 250, starting at 250.
*retry_count* sets the number of retries when an ack fails

#### get_retry_delay()
Returns the delay factor or -1 on error

#### get_retry_count()
Returns the retry count or -1 on error

### Channel settings
Channels are set from 2.4GHz to 2.525GHz in 1 MHz increments
Note that not all channels are allowed/valid in all countries.

#### set_channel(uint8_t channel)
Set a channel value from 0 to 124. Invalid channels will result in a failure and false returned.
If set then function returns true

#### get_channel()
Returns the channel set in the hardware. Returns 0x80 on error.

### Setup register
These set of functions update automatically if auto_update() is set to true.
They operate on the setup register held in the hardware.

#### read_setup()
This is a manual read function to pull settings from the device.
If auto_update() is enabled then this will be called automatically.
Returns false if call fails to call the hardware.

#### write_setup()
This is a manual write function to push settings to the device.
If auto_update() is enabled then this will be called automatically.
Returns false if call fails to call the hardware.

#### set_power_level(uint8_t level)
Power level can be one of 4 values:
- RF24_0DBM which is max power 0db
- RF24_NEG6DBM which is -6db power
- RF24_NEG12DBM which is -12db power
- RF24_NEG18DBM which is the lowest power at -18db

Invalid values are ignored

#### get_power_level()
Returns one of the following values
- RF24_0DBM
- RF24_NEG6DBM
- RF24_NEG12DBM
- RF24_NEG18DBM

If an error occurs then the value may be inaccurate

#### set_data_rate(uint8_t datarate)
*datarate* represents one of the following values
- RF24_250KBPS
- RF24_1MBPS
- RF24_2MBPS

Invalid values will default the rate to 1MB per sec

#### get_data_rate()
Returns the data rate set in the hardware
- RF24_250KBPS
- RF24_1MBPS
- RF24_2MBPS

If an error occurs then the value may be inaccurate

#### set_continuous_carrier_transmit(bool set)
Used for testing by setting a continuous carrier wave.
Set to true along with PLL lock enabled to test.
See Appendix C in the product specification.

#### is_continuous_carrier_transmit()
Returns true if continuous carrier wave is enabled. False if not set.


