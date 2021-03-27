package main

// This is the most minimal blinky example and should run almost everywhere.
//
import (
	"device/stm32"
	"machine"
	"time"

	"tinygo.org/x/drivers/lora/sx126x"
)

/*
const (
	//GET
	RADIO_GET_STATUS         = uint8(0xC0)
	RADIO_GET_PACKETTYPE     = uint8(0x11)
	RADIO_GET_RXBUFFERSTATUS = uint8(0x13)
	RADIO_GET_PACKETSTATUS   = uint8(0x14)
	RADIO_GET_RSSIINST       = uint8(0x15)
	RADIO_GET_STATS          = uint8(0x10)
	RADIO_GET_IRQSTATUS      = uint8(0x12)
	RADIO_GET_ERROR          = uint8(0x17)
	// SET
	RADIO_SET_FR_FREQUENCY = uint8(0x86)
	RADIO_SET_SLEEP        = uint8(0x84)
	RADIO_SET_TX           = uint8(0x83)
	RADIO_SET_STANDBY      = uint8(0x80)
	RADIO_SET_PACKETTYPE   = uint8(0x8A)
)
*/
// initRadio prepares SubGhz radio for operation
// RM0461 4.9
func SubGhzInit() error {

	// TODO : VOS

	// Init SubGhz SPI
	stm32.RCC.APB3ENR.SetBits(stm32.RCC_APB3ENR_SUBGHZSPIEN)
	_ = stm32.RCC.APB3ENR.Get() // Delay after RCC periph clock enable

	// Enable TXCO and HSE
	// Already done in runtime_stm32wl .... FIXME
	stm32.RCC.CR.SetBits(stm32.RCC_CR_HSEBYPPWR)
	stm32.RCC.CR.SetBits(stm32.RCC_CR_HSEON)
	for !stm32.RCC.CR.HasBits(stm32.RCC_CR_HSERDY) {
	}

	// Reset SUBGHZ device
	stm32.RCC.CSR.SetBits(stm32.RCC_CSR_RFRST)
	time.Sleep(time.Millisecond * 10)
	stm32.RCC.CSR.ClearBits(stm32.RCC_CSR_RFRST)
	for stm32.RCC.CSR.HasBits(stm32.RCC_CSR_RFRSTF) {
	}

	// Disable radio : SUBGHZ SPI NSS signal is at level high (RM0461-5.5.18)
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)

	// Enable Exti Line 44: Radio IRQ ITs for CPU1 (RM0461-14.3.1)
	stm32.EXTI.IMR2.SetBits(1 << 12) // IM44

	// Clear busy flag
	stm32.PWR.SCR.SetBits(stm32.PWR_SCR_CWRFBUSYF)

	// Enable radio busy wakeup from Standby for CPU (RM0461-5.5.3)
	//stm32.PWR.CR3.SetBits(stm32.PWR_CR3_EWRFBUSY)

	// Clear wakeup radio busy flag (RM0461-5.5.7)
	//stm32.PWR.SCR.SetBits(stm32.PWR_SCR_CWRFBUSYF)

	//  SUBGHZSPI configuration (Prescaler /8)
	stm32.SPI3.CR1.ClearBits(stm32.SPI_CR1_SPE)                                                  // Disable SPI
	stm32.SPI3.CR1.Set(stm32.SPI_CR1_MSTR | stm32.SPI_CR1_SSI | (0b10 << 3) | stm32.SPI_CR1_SSM) // Software Slave Management (NSS)
	stm32.SPI3.CR2.Set(stm32.SPI_CR2_FRXTH | (0b111 << 8))                                       // FIFO Threshold and 8bit size
	stm32.SPI3.CR1.SetBits(stm32.SPI_CR1_SPE)                                                    // Enable SPI

	return nil
}

func xstatus(lora sx126x.Device) {
	dat := lora.GetStatus()
	chipmode := (dat & (0x7 << 4)) >> 4
	cmdstatus := (dat & (0x7 << 1)) >> 1
	println("status:", dat, "cmd:", cmdstatus, " mode:", chipmode)
}

// DS.SX1261-2.W.APP section 14.2 p99
func configure(lora sx126x.Device) {

	msg := []byte("Hello TinyGo!")
	// Standby mode for configuration
	lora.Standby()

	println("Clear IRQ")
	// Clear Interrupt Flag
	lora.ClearIrqStatus(sx126x.SX126X_IRQ_ALL)

	println("SetPacketType")
	// Define the protocol
	lora.SetPacketType(sx126x.SX126X_PACKET_TYPE_LORA)

	println("SetRfFrequency")
	// Define the RF Frequency
	lora.SetRfFrequency(868000000)

	println("SetPaConfig")
	// Set Power Amplifier
	lora.SetPaConfig(0x04, 0x07, 0x00, 0x01)

	println("SetTxParams")
	// Set output power and ramping time (0x16:HP22Db)
	lora.SetTxParams(0x16, sx126x.SX126X_PA_RAMP_200U) // CHECKME

	println("SetBaseAddress")
	// Were to store the payload in buffer
	lora.SetBufferBaseAddress(0, 0)

	println("WriteBuffer")
	// write the payload
	lora.WriteBuffer(msg)

	println("SetModulation")
	// Set modulation params
	// SF7 / 125 KHz / CR 4/7 / No Optimis
	lora.SetModulationParams(7, sx126x.SX126X_LORA_BW_125_0, sx126x.SX126X_LORA_CR_4_7, sx126x.SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF)

	println("SetPacketParam")
	// Define frame format
	//10 preamble bits, 8bits preamble detection,
	lora.SetPacketParam(10, 0x04, uint8(len(msg)), sx126x.SX126X_LORA_CRC_ON, sx126x.SX126X_LORA_IQ_INVERTED)

	// Configure DIO / IRQ
	// TODO

	// Set Sync Word (0x34?)
	// TODO
	/*
		println("Start TX")
		// Start transmission
		lora.SetTx(sx126x.SX126X_TX_TIMEOUT_NONE)

		println("Wait IRQ")
		// Wait IRQ TxDone or Timeout
		for {
			st := lora.GetIrqStatus()
			time.Sleep(1000 * time.Millisecond)
			print("IRQ Status:", st)
		}
		println("TX DONE")

		println("ClearIrqStatus")
		// Clear Interrupt Flag
		lora.ClearIrqStatus(sx126x.SX126X_IRQ_TX_DONE)
	*/
}

func main() {

	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// UART0 (Console)
	machine.UART0.Configure(machine.UARTConfig{TX: machine.UART_TX_PIN, RX: machine.UART_TX_PIN, BaudRate: 9600})

	println("STM32WL Radio Init Example")

	// Init radio module
	SubGhzInit()

	// Attach driver
	lora := sx126x.New(machine.SPI0)

	// Switch to Sleep, then Standby
	lora.Sleep()
	lora.Standby()
	println("* Now in Standby")

	/*
		lora.SetPacketType(0x01)
		println("2")
		a := lora.GetPacketType()
		println("Read Packet Type: ", a)
	*/
	xstatus(lora)

	println("SET TX MODE: ")
	lora.SetTx(0)

	xstatus(lora)

	//startTransmit(lora)

}
