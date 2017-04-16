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


