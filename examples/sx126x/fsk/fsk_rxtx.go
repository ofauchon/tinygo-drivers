package main

// In this example, a FSK packet will be sent every 10s
// module will be in RX mode between two transmissions

import (
	"machine"
	"time"

	"tinygo.org/x/drivers/sx126x"
)

const (
	RXTIMEOUT_MS = 1000
	TXTIMEOUT_MS = 5000
)

var (
	radio *sx126x.Device
	txmsg = []byte("Hello TinyGO")
)

func main() {
	time.Sleep(3 * time.Second)

	println("\n# TinyGo FSK RX/TX test")
	println("# ----------------------")
	machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Create the driver
	radio = sx126x.New(spi)
	radio.SetDeviceType(sx126x.DEVICE_TYPE_SX1262)

	// Create radio controller for target
	radio.SetRadioController(newRadioControl())

	// Detect the device
	if state := radio.DetectDevice(); !state {
		panic("sx126x not detected.")
	}

	// Configure radio
	radio.SetPacketType(sx126x.SX126X_PACKET_TYPE_GFSK)
	radio.SetFrequency(868000)
	radio.SetPreambleLength(512)
	radio.SetTxPower(20)
	radio.SetCurrentLimit(20)
	radio.SetSyncWord(0x7fd9)
	//	radio.setRxBoostedGainMode(true)
	//radio.SetBitRate(38.4)
	radio.SetFrequencyDeviation(19.2)
	//radio.setDataShaping(RADIOLIB_SHAPING_NONE)
	//radio.SetEncoding(RADIOLIB_ENCODING_NRZ)
	//radio.SetRxBandwidth(234.3) //312.0); //(250.0);
	//radio.SetCRC(0, 0x0000, 0x8408, false)
	//radio.DisableAddressFiltering()

	var count uint
	for {
		start := time.Now()

		println("main: Receiving FSK for 10 seconds")
		for time.Since(start) < 10*time.Second {
			buf, err := radio.Rx(RXTIMEOUT_MS)
			if err != nil {
				println("RX Error: ", err)
			} else if buf != nil {
				println("Packet Received: len=", len(buf), string(buf))
			}
		}
		println("main: End FSK RX")
		println("FSK TX size=", len(txmsg), " -> ", string(txmsg))
		err := radio.Tx(txmsg, TXTIMEOUT_MS)
		if err != nil {
			println("TX Error:", err)
		}
		count++
	}

}
