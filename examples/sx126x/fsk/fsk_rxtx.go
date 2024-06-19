package main

// In this example, a FSK packet will be sent every 10s
// module will be in RX mode between two transmissions

import (
	"log"
	"machine"
	"time"

	"tinygo.org/x/drivers/sx126x"
)

const (
	RXTIMEOUT_MS = 2000
	TXTIMEOUT_MS = 5000

	IOHC_RADIO1 = 868250
	IOHC_RADIO2 = 868950
	IOHC_RADIO3 = 869850
)

var (
	radio *sx126x.Device
	txmsg = []byte("Hello TinyGO")
)

// check() handle error and fatal if not nil
func check(key string, e error) {
	if e != nil {
		log.Fatal("FATAL: ", key, ":", e)
	}
}

func main() {

	println("\n# TinyGo FSK RX/TX test")
	println("# ----------------------")
	machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Create the driver
	radio = sx126x.New(spi)

	// RadioControl will handle board-specific radio HW (eg:SPI)
	radio.SetRadioController(newRadioControl())

	// Ensure the radio module is connected
	if state := radio.DetectDevice(); !state {
		panic("sx126x not detected.")
	}

	// Prepare radio for FSK communications
	check("begin_fsk", radio.BeginFSK())

	// Custom Radio configuration

	check("SetFrequency", radio.SetFrequency(IOHC_RADIO2))
	check("SetPreambleLength", radio.SetPreambleLength(512))
	check("SetTxPower", radio.SetTxPower(20))
	check("SetCurrentLimit", radio.SetCurrentLimit(100))
	check("SetBitRate", radio.SetBitRate(38.4))
	check("SetFrequencyDeviation", radio.SetFrequencyDeviation(19.2))
	check("SetDataShaping", radio.SetDataShaping(sx126x.SX126X_GFSK_FILTER_NONE))
	check("SetRxBandwidth", radio.SetRxBandwidth(234.3))
	check("SetCrc", radio.SetCrc(0, 0x0000, 0x8408, false))
	check("DisableAddressFiltering", radio.DisableAddressFiltering())
	check("SetWhitening", radio.SetWhitening(false, 0x00))
	check("SetPacketType", radio.SetPacketMode(sx126x.SX126X_GFSK_PACKET_VARIABLE, sx126x.SX126X_MAX_PACKET_LENGTH))
	check("SetSyncWord", radio.SetSyncWord([]uint8{0x7f, 0xd9}))

	println("Radio configuration done")
	println("ERRORS:", radio.GetDeviceErrors())

	println("FSK TX size=", len(txmsg), " -> ", string(txmsg))
	err := radio.Tx(txmsg, TXTIMEOUT_MS)
	if err != nil {
		println("TX Error:", err)
	}
	println("ERRORS:", radio.GetDeviceErrors())

	var count uint
	for {
		start := time.Now()

		println("main: Receiving FSK for 60 seconds")
		for time.Since(start) < 60*time.Second {
			buf, err := radio.Rx(RXTIMEOUT_MS)
			if err != nil {
				println("RX Error: ", err)
			} else if len(buf) == 0 {
				println("Packet Empty")
			} else if len(buf) > 0 {
				println("Packet Received: len=", len(buf), string(buf))
			}
		}
		println("main: End FSK RX")

		count++

	}

}
