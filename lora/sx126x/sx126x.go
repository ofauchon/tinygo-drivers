// Package sx126x provides a driver for SX126x LoRa transceivers.

package sx126x

import (
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
	//  SPIreadCommand(SX126X_CMD_GET_STATUS, &data, 1);
	r := d.ExecGetCommand(SX126X_CMD_GET_STATUS, 1)
	return r[1]
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
	d.ExecSetCommand(SX126X_CMD_SET_SLEEP, []uint8{0})
}

// Sleep switch device to standby mode
func (d *Device) Standby() {
	d.ExecSetCommand(SX126X_CMD_SET_STANDBY, []uint8{0})
}

// SetRfFrequency sets the radio frequency
func (d *Device) SetRfFrequency(f uint32) {
	var p [4]uint8
	p[0] = uint8((f >> 24) & 0xFF)
	p[1] = uint8((f >> 16) & 0xFF)
	p[2] = uint8((f >> 8) & 0xFF)
	p[3] = uint8((f >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_RF_FREQUENCY, p[:])
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
	//	d.csPin.Low()
	d.spi.Tx([]byte{reg & 0x7f}, nil)
	var value [1]byte
	d.spi.Tx(nil, value[:])
	//	d.csPin.High()
	return value[0]
}

// WriteRegister writes value to register
func (d *Device) WriteRegister(reg uint8, value uint8) uint8 {
	var response [1]byte
	//	d.csPin.Low()
	d.spi.Tx([]byte{reg | 0x80}, nil)
	d.spi.Tx([]byte{value}, response[:])
	//	d.csPin.High()
	return response[0]
}

// ExecCommand send a command to configure the peripheral
func (d *Device) ExecSetCommand(cmd uint8, buf []uint8) {
	//	d.csPin.Low()
	d.spi.Tx([]byte{cmd}, nil)
	for i := 0; i < len(buf); i++ {
		d.spi.Tx([]byte{buf[i]}, nil) // Send options
	}
	//	d.csPin.High()
}

// SpiExecCommand send a command to configure the peripheral
func (d *Device) ExecGetCommand(cmd uint8, size int) []uint8 {
	var buf []uint8
	//	d.csPin.Low()
	d.spi.Tx([]byte{cmd}, nil)
	d.spi.Tx([]byte{0x00}, nil)
	var value [1]byte
	for i := 0; i < size; i++ {
		d.spi.Tx(nil, value[:])
		buf = append(buf, value[0]) // Read bytes
	}
	//d.csPin.High()
	return buf
}
