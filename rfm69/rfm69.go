// Package rfm69 implements a driver for the RFM69 module.
//
// This code was inspired from https://github.com/ahessling/RFM69-STM32/blob/master/rfm69.cpp
//
package rfm69 // import "tinygo.org/x/drivers/rfm69"

import (
	"errors"
	"machine"
	"time"
)

const (
	RFM69_MODE_SLEEP   uint8 = 0 //!< Sleep mode (lowest power consumption)
	RFM69_MODE_STANDBY uint8 = 1 //!< Standby mode
	RFM69_MODE_SYNTH   uint8 = 2 //!< Frequency synthesizer enabled
	RFM69_MODE_TX      uint8 = 3 //!< TX mode (carrier active)
	RFM69_MODE_RX      uint8 = 4 //!< RX mode
)
const RFM69_FSTEP = 61.03515625
const COURSE_TEMP_COEF = int8(-90)

// Device wraps an SPI connection.
type Device struct {
	spiBus     machine.SPI
	rstPin     machine.Pin
	nssPin     machine.Pin
	buffer     []byte
	bufferSize int16
	mode       uint8
	isRFM69HW  bool
	frequency  uint32
}

type Config struct {
}

// New creates a new RFM69 device. The SPI bus and GPIOs must already be configured.
func New(bus machine.SPI, rstPin machine.Pin, nssPin machine.Pin, isRFM69HW bool) *Device {
	ret := &Device{
		spiBus:    bus,
		rstPin:    rstPin,
		nssPin:    nssPin,
		mode:      RFM69_MODE_STANDBY,
		isRFM69HW: isRFM69HW,
		frequency: 0,
	}

	return ret
}

// Reset reinitialize the device by toggling RST line
// Specs says RST pin
func (d *Device) Reset() {
	d.rstPin.High()
	time.Sleep(1 * time.Millisecond)
	d.rstPin.Low()
	time.Sleep(10 * time.Millisecond)
	d.mode = RFM69_MODE_STANDBY
}

// readReg reads value of register
func (d *Device) ReadReg(reg byte) (byte, error) {

	if reg > 0x7F {
		return 0, errors.New("reg>0x7F Aborting")
	}

	d.nssPin.Low()

	ret, err := d.spiBus.Transfer(reg)
	if err != nil {
		return 0, err
	}
	ret, err = d.spiBus.Transfer(0)
	if err != nil {
		return 0, err
	}
	d.nssPin.High()
	return ret, nil
}

// WriteReg writes val to register
func (d *Device) WriteReg(reg, val byte) error {

	if reg > 0x7F {
		return errors.New("reg>0x7F Aborting")
	}

	d.nssPin.Low()

	_, err := d.spiBus.Transfer(reg | 0x80)
	if err != nil {
		return err
	}
	_, err = d.spiBus.Transfer(val)
	if err != nil {
		return err
	}
	d.nssPin.High()
	return nil
}

// SetHighPower turns on amps on RFM69HW/SX1231H hardware
func (d *Device) SetHighPower(setHigh bool) {
	v, _ := d.ReadReg(REG_PALEVEL)
	if setHigh {
		d.WriteReg(REG_TESTPA1, (v&0x1F)|0x40)
		d.WriteReg(REG_TESTPA1, 0x5D)
		d.WriteReg(REG_TESTPA2, 0x7C)
	} else {
		d.WriteReg(REG_TESTPA1, (v&0x1F)|0x80)
		d.WriteReg(REG_TESTPA1, 0x55)
		d.WriteReg(REG_TESTPA2, 0x70)
	}
}

// SetMode sets operation mode
func (d *Device) SetMode(newMode uint8) error {
	/*
		if newMode == d.mode || newMode > RFM69_MODE_RX {
			return nil
		}
	*/
	r, _ := d.ReadReg(REG_OPMODE)

	switch newMode {
	case RFM69_MODE_TX:
		d.WriteReg(REG_OPMODE, (r&0xE3)|RF_OPMODE_TRANSMITTER)
		if d.isRFM69HW {
			d.SetHighPower(true)
		}
	case RFM69_MODE_RX:
		d.WriteReg(REG_OPMODE, (r&0xE3)|RF_OPMODE_RECEIVER)
		if d.isRFM69HW {
			d.SetHighPower(false)
		}
	case RFM69_MODE_SYNTH:
		d.WriteReg(REG_OPMODE, (r&0xE3)|RF_OPMODE_SYNTHESIZER)
	case RFM69_MODE_STANDBY:
		d.WriteReg(REG_OPMODE, (r&0xE3)|RF_OPMODE_STANDBY)
	case RFM69_MODE_SLEEP:
		d.WriteReg(REG_OPMODE, (r&0xE3)|RF_OPMODE_SLEEP)
	}

	d.mode = newMode
	return nil
}

// GetMode returns operation mode
func (d *Device) GetMode() uint8 {

	return d.mode

}

// WaitForMode waits for operation mode to be changed (or timeout)
func (d *Device) WaitForMode() error {
	//	println("waitformode start")
	var loop = uint(0)
	for {
		reg, err := d.ReadReg(REG_IRQFLAGS1)
		if err != nil {
			return err
		}
		if loop > 1000 {
			println("WaitForMode Timeout")
			return errors.New("WaitForMode Timeout")
		}
		if (reg & RF_IRQFLAGS1_MODEREADY) > 0 {
			//			println("waitformode ok")
			return nil
		}

		loop++
		time.Sleep(1 * time.Millisecond)
	}
}

// ReadTemperature returns current RFM69 temperature
func (d *Device) ReadTemperature(calFactor uint8) (int8, error) {

	if d.mode != RFM69_MODE_STANDBY && d.mode != RFM69_MODE_SYNTH {
		return 0, errors.New("ReadTemperature only allowed in stdby and synth modes ")
	}
	d.WriteReg(uint8(REG_TEMP1), uint8(RF_TEMP1_MEAS_START))

	var temp uint8
	var err error

	count := 0
	for {
		temp, err = d.ReadReg(uint8(REG_TEMP1))
		if err != nil {
			return 0, err
		}
		time.Sleep(1 * time.Millisecond)

		if (temp & RF_TEMP1_MEAS_RUNNING) == 0 {
			break
		}
		if count > 10 { // Datasheet says measurement takes less than 100us ...
			return 0, errors.New("Read temperature timeout")
		}
		count++
	}

	temp, err = d.ReadReg(uint8(REG_TEMP2))
	if err != nil {
		return 0, err
	}
	ret := int8(^temp) + COURSE_TEMP_COEF + int8(calFactor) //'complement'corrects the slope, rising temp = rising val
	return ret, nil
}

//SetTxPower sets TX power level ( 0 = min .. 31 = max )
func (d *Device) SetTxPower(power uint8) {
	if power > 31 {
		power = 31
	}
	cur, _ := d.ReadReg(REG_PALEVEL)
	d.WriteReg(REG_PALEVEL, (cur&0xE0)|power)
}

//ReadRSSI returns current RSSI level
func (d *Device) ReadRSSI(forceTrigger bool) uint8 {

	if forceTrigger {
		d.WriteReg(REG_RSSICONFIG, RF_RSSI_START)
		for {
			v, _ := d.ReadReg(REG_RSSICONFIG)
			if (v & RF_RSSI_DONE) == 0x00 {
				break
			}
			// Wait
		}
	}
	rssi, err := d.ReadReg(uint8(REG_RSSIVALUE))
	if err != nil {
		return 0
	}
	return ^rssi
}

// SetFrequency changes RFM69 the current frequency
func (d *Device) SetFrequency(freq uint32) {
	if d.frequency == freq {
		return
	}
	/*
		if d.mode == RFM69_MODE_TX {
			d.SetMode(RFM69_MODE_RX)
		}
	*/
	f := float64(freq)
	f = f / RFM69_FSTEP
	freq = uint32(f)

	d.WriteReg(REG_FRFMSB, uint8(freq>>16))
	d.WriteReg(REG_FRFMID, uint8(freq>>8))
	d.WriteReg(REG_FRFLSB, uint8(freq))

	d.frequency = freq
}

// GetFrequency returns the current frequency (in Hz)
func (d *Device) GetFrequency() uint32 {
	var freq uint32
	var r1, r2, r3 uint8
	r1, _ = d.ReadReg(uint8(REG_FRFMSB))
	r2, _ = d.ReadReg(uint8(REG_FRFMID))
	r3, _ = d.ReadReg(uint8(REG_FRFLSB))

	freq = uint32(r1) << 16
	freq += uint32(r2) << 8
	freq += uint32(r3)
	freq = uint32(float64(freq) * RFM69_FSTEP)
	return freq
}

//configure RFM69 device
func (d *Device) Configure(config [][]byte) error {
	for _, c := range config {
		err := d.WriteReg(c[0], c[1])
		if err != nil {
			return err
		}
		v, _ := d.ReadReg(c[0])
		if v != c[1] {
			println("Verification failed reg:", c[0], " -> ", v, " != ", c[1])
			//return errors.New("Verification failed")
		}
	}
	println("Config OK ")
	return nil
}

func (d *Device) ClearFifo() error {
	err := d.WriteReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN)
	if err != nil {
		return err
	}
	return nil
}

// Send takes data buffer to the chipset for transmission
func (d *Device) Send(data []byte) error {
	if len(data) == 0 || len(data) > 255 {
		errors.New("invalid size length of packet")
	}

	/*
		if d.mode != RFM69_MODE_SLEEP {
			println("Send => set standby")
			d.SetMode(RFM69_MODE_STANDBY)
			d.WaitForMode()
		}
	*/
	d.ClearFifo()

	println("Will send bulk")
	// Bulk transfert
	d.nssPin.Low()
	d.spiBus.Transfer(REG_FIFO | 0x80)
	d.spiBus.Transfer(uint8(len(data) + 3))
	d.spiBus.Transfer(uint8(0x22))
	d.spiBus.Transfer(uint8(0x04))

	for i := 0; i < len(data); i++ {
		d.spiBus.Transfer(data[i])
	}
	d.nssPin.High()
	d.SetMode(RFM69_MODE_TX)
	println("Bulk TX DONE ")

	cnt := uint(0)
	for {
		status, _ := d.ReadReg(REG_IRQFLAGS2)
		//println("read ", status)
		if (status & RF_IRQFLAGS2_PACKETSENT) > 0 {
			break
		}
		if cnt > 1000 {
			return errors.New("Send timeout")
		}
		cnt++
		time.Sleep(1 * time.Millisecond)
	}
	println("Pcket sent ok in   ", cnt, "ms")

	//d.SetMode(RFM69_MODE_STANDBY)
	return nil
}

func (d *Device) Receive() ([]uint8, error) {

	var data []uint8
	m := d.GetMode()
	if m == RFM69_MODE_RX {
		a, _ := d.ReadReg(REG_IRQFLAGS2)
		if (a & RF_IRQFLAGS2_PAYLOADREADY) > 0 {

			d.SetMode(RFM69_MODE_STANDBY)
			/*
				if (a & RF_IRQFLAGS2_CRCOK) > 0 {
					println("CRC OK")
				} else {
					println("CRC NOT OK")
				}
			*/
			d.nssPin.Low()
			d.spiBus.Transfer(REG_FIFO)
			payloadLen, _ := d.spiBus.Transfer(0)
			//println("INT: Payloadlen:", payloadLen)

			for i := uint8(0); i < payloadLen; i++ {
				c, _ := d.spiBus.Transfer(0)
				data = append(data, c)
				//println("INT: DATA:", c)
			}

		} else {
			println("Int: Not payloadready")
		}
		d.SetMode(RFM69_MODE_RX) // switch back RX
	} else {
		println("Int: But not in RX. Discard")
	}
	return data, nil

}
