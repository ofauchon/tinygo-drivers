// Package sx126x provides a driver for SX126x LoRa transceivers.

package sx126x

import (
	"device/stm32"
	"machine"
)

// RadioEvent are send to application through a channel
type RadioEvent struct {
	EventType int
	EventData []byte
}

const (
	EventRxDone    = iota
	EventTxDone    = iota
	EventRxTimeout = iota
)

// Device wraps an SPI connection to a SX127x device.
type Device struct {
	spi            machine.SPI
	radioEventChan chan RadioEvent
	cnf            Config
}

// Config holds the LoRa configuration parameters
type Config struct {
}

// New creates a new SX126x connection. The SPI bus must already be configured.
func New(spi machine.SPI) Device {
	k := Device{
		spi:            spi,
		radioEventChan: make(chan RadioEvent, 10),
	}
	return k
}

// --------------------------------------------------
// Radio-specific functions
// --------------------------------------------------

// GetStatus returns radio status
func (d *Device) GetStatus() uint8 {
	r := d.ExecGetCommand(SX126X_CMD_GET_STATUS, 1)
	return r[0]
}

// GetIrqStatus returns radio status
func (d *Device) GetIrqStatus() uint16 {
	r := d.ExecGetCommand(SX126X_CMD_GET_STATUS, 3)
	ret := uint16(r[0]<<8 | r[1])
	return ret
}

// SetTx enable Tx Mode with Tx Timeout
func (d *Device) SetTx(t uint32) {
	var p [3]uint8
	p[0] = uint8((t >> 16) & 0xFF)
	p[1] = uint8((t >> 8) & 0xFF)
	p[2] = uint8((t >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_TX, p[:])
}

// Sleep switch device to sleep mode
func (d *Device) Sleep() {
	d.ExecSetCommand(SX126X_CMD_SET_SLEEP, []uint8{SX126X_SLEEP_START_WARM | SX126X_SLEEP_RTC_OFF})
}

// Standby switch device to RC standby mode
func (d *Device) Standby() {
	d.ExecSetCommand(SX126X_CMD_SET_STANDBY, []uint8{SX126X_STANDBY_RC})
}

// SetPacketType sets the packet type
func (d *Device) SetPacketType(packetType uint8) {
	var p [1]uint8
	p[0] = packetType
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_TYPE, p[:])
}

// SetPacketParam sets various packet-related params
func (d *Device) SetPacketParam(preambleLength uint16, crcType, payloadLength, headerType, invertIQ uint8) {
	var p [6]uint8
	p[0] = uint8((preambleLength >> 8) & 0xFF)
	p[1] = uint8(preambleLength & 0xFF)
	p[2] = headerType
	p[3] = payloadLength
	p[4] = crcType
	p[5] = invertIQ
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_PARAMS, p[:])
}

// SetBufferBaseAddress sets base address for buffer
func (d *Device) SetBufferBaseAddress(txBaseAddress, rxBaseAddress uint8) {
	var p [2]uint8
	p[0] = txBaseAddress
	p[1] = rxBaseAddress
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_PARAMS, p[:])
}

// SetRfFrequency sets the radio frequency
func (d *Device) SetRfFrequency(frequency uint32) {
	var p [4]uint8
	freq := uint32(float64(frequency) / float64(SX126X_FREQUENCY_STEP_SIZE))
	p[0] = uint8((freq >> 24) & 0xFF)
	p[1] = uint8((freq >> 16) & 0xFF)
	p[2] = uint8((freq >> 8) & 0xFF)
	p[3] = uint8((freq >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_RF_FREQUENCY, p[:])
}

// SetPaConfig sets the Power Amplifier configuration
func (d *Device) SetPaConfig(paDutyCycle, hpMax, deviceSel, paLut uint8) {
	var p [4]uint8
	p[0] = paDutyCycle
	p[1] = hpMax
	p[2] = deviceSel
	p[3] = paLut
	d.ExecSetCommand(SX126X_CMD_SET_PA_CONFIG, p[:])
}

// SetTxConfig sets power and rampup time
func (d *Device) SetTxConfig(power, rampTime uint8) {
	var p [2]uint8
	p[0] = power
	p[1] = rampTime
	d.ExecSetCommand(SX126X_CMD_SET_TX_PARAMS, p[:])
}

// SetModulationParams sets the Lora modulation frequency
func (d *Device) SetModulationParams(spreadingFactor, bandwidth, codingRate, lowDataRateOptimize uint8) {
	var p [4]uint8
	p[0] = spreadingFactor
	p[1] = bandwidth
	p[2] = codingRate
	p[3] = lowDataRateOptimize
	d.ExecSetCommand(SX126X_CMD_SET_MODULATION_PARAMS, p[:])
}

// ClearIrqStatus clears IRQ flags
func (d *Device) ClearIrqStatus(clearIrqParams uint16) {
	var p [2]uint8
	p[0] = uint8((clearIrqParams >> 8) & 0xFF)
	p[1] = uint8(clearIrqParams & 0xFF)
	d.ExecSetCommand(SX126X_CMD_CLEAR_IRQ_STATUS, p[:])
}

// WriteBuffer sets base address for buffer
func (d *Device) WriteBuffer(data []uint8) {
	p := []uint8{0} // Zero offset
	p = append(p, data...)
	d.ExecSetCommand(SX126X_CMD_WRITE_BUFFER, p[:])
}

// --------------------------------------------------
// Internal functions
// --------------------------------------------------

// Reset re-initialize the sx127x device
// sx1276 have nRST pin
/*
func (d *Device) Reset() {
	d.rstPin.Low()
	time.Sleep(100 * time.Millisecond)
	d.rstPin.High()
	time.Sleep(100 * time.Millisecond)
}
*/

// ReadRegister reads register value
func (d *Device) ReadRegister(reg uint8) uint8 {
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)
	//	d.csPin.Low()
	d.spi.Tx([]byte{reg & 0x7f}, nil)
	var value [1]byte
	d.spi.Tx(nil, value[:])
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
	//	d.csPin.High()
	return value[0]
}

// WriteRegister writes value to register
func (d *Device) WriteRegister(reg uint8, value uint8) uint8 {
	var response [1]byte
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)
	//	d.csPin.Low()
	d.spi.Tx([]byte{reg | 0x80}, nil)
	d.spi.Tx([]byte{value}, response[:])
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
	//	d.csPin.High()
	return response[0]
}

// ExecSetCommand send a command to configure the peripheral
func (d *Device) ExecSetCommand(cmd uint8, buf []uint8) {
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)
	//	d.csPin.Low()
	d.spi.Tx([]byte{cmd}, nil)
	d.spi.Tx(buf, nil)
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
	//	d.csPin.High()
}

// ExecGetCommand queries the peripheral the peripheral
func (d *Device) ExecGetCommand(cmd uint8, size int) []uint8 {
	var buf []uint8
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)
	//	d.csPin.Low()
	d.spi.Tx([]byte{cmd}, nil)
	d.spi.Tx([]byte{0x00}, nil) // Drop first status byte
	var value [1]byte
	for i := 0; i < size; i++ {
		d.spi.Tx(nil, value[:])
		buf = append(buf, value[0]) // Read bytes
	}
	//d.csPin.High()
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
	return buf
}
