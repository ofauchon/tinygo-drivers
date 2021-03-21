// Package sx126x provides a driver for SX126x LoRa transceivers.

package sx126x

import (
	"machine"
	"time"
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
	csPin          machine.Pin
	rstPin         machine.Pin
	radioEventChan chan RadioEvent
	cnf            Config
}

// Config holds the LoRa configuration parameters
type Config struct {
}

// New creates a new SX127x connection. The SPI bus must already be configured.
func New(spi machine.SPI, csPin machine.Pin, rstPin machine.Pin) Device {
	k := Device{
		spi:            spi,
		csPin:          csPin,
		rstPin:         rstPin,
		radioEventChan: make(chan RadioEvent, 10),
	}

	return k
}

// Reset re-initialize the sx127x device
// sx1276 have nRST pin
func (d *Device) Reset() {
	d.rstPin.Low()
	time.Sleep(100 * time.Millisecond)
	d.rstPin.High()
	time.Sleep(100 * time.Millisecond)
}

// ReadRegister reads register value
func (d *Device) ReadRegister(reg uint8) uint8 {
	d.csPin.Low()
	d.spi.Tx([]byte{reg & 0x7f}, nil)
	var value [1]byte
	d.spi.Tx(nil, value[:])
	d.csPin.High()
	return value[0]
}

// WriteRegister writes value to register
func (d *Device) WriteRegister(reg uint8, value uint8) uint8 {
	var response [1]byte
	d.csPin.Low()
	d.spi.Tx([]byte{reg | 0x80}, nil)
	d.spi.Tx([]byte{value}, response[:])
	d.csPin.High()
	return response[0]
}

// ExecCommand send a command to configure the peripheral
func (d *Device) ExecSetCommand(cmd uint8, buf []uint8) {
	d.csPin.Low()
	d.spi.Tx(cmd, nil)
	for i := 0; i < len(buf); i++ {
		d.spi.Tx(buf[i], nil) // Send options
	}
	d.csPin.High()
}

// SpiExecCommand send a command to configure the peripheral
func (d *Device) ExecGetCommand(cmd uint8, size int) []uint8 {
	var buf []uint8
	d.csPin.Low()
	d.spi.Tx(cmd, nil)
	d.spi.Tx(0x00, nil)
	var value [1]byte
	for i := 0; i < size; i++ {
		d.spi.Tx(nil, value[:])
		buf = append(buf, value[0]) // Read bytes
	}
	d.csPin.High()
	return buf
}

func (d *Device) SetRfFrequency(f uint32) {
	var p [4]uint8
	p[0] = uint8((f >> 24) & 0xFF)
	p[1] = uint8((f >> 16) & 0xFF)
	p[2] = uint8((f >> 8) & 0xFF)
	p[3] = uint8((f >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_RF_FREQUENCY, p)
}
