package main

// This is the most minimal blinky example and should run almost everywhere.

import (
	"device/stm32"
	"machine"
	"time"

	"tinygo.org/x/drivers/lora/sx126x"
)

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

// initRadio prepares SubGhz radio for operation
// RM0461 4.9
func initRadio() {

	// TODO : VOS

	// Init SubGhz SPI
	stm32.RCC.APB3ENR.SetBits(stm32.RCC_APB3ENR_SUBGHZSPIEN)

	// Reset SUBGHZ device
	stm32.RCC.CSR.SetBits(stm32.RCC_CSR_RFRST)
	stm32.RCC.CSR.ClearBits(stm32.RCC_CSR_RFRST)
	for stm32.RCC.CSR.HasBits(stm32.RCC_CSR_RFRSTF) {
	}

	// Disable radio : SUBGHZ SPI NSS signal is at level high (RM0461-5.5.18)
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)

	// Enable Exti Line 44: Radio IRQ ITs for CPU1 (RM0461-14.3.1)
	stm32.EXTI.IMR2.SetBits(1 << 12) // IM44

	// Enable radio busy wakeup from Standby for CPU (RM0461-5.5.3)
	stm32.PWR.CR3.SetBits(stm32.PWR_CR3_EWRFBUSY)

	// Clear wakeup radio busy flag (RM0461-5.5.7)
	stm32.PWR.SCR.SetBits(stm32.PWR_SCR_CWRFBUSYF)

	//  SUBGHZSPI configuration (Prescaler /8)
	stm32.SPI3.CR1.ClearBits(stm32.SPI_CR1_SPE)                                                  // Disable SPI
	stm32.SPI3.CR1.Set(stm32.SPI_CR1_MSTR | stm32.SPI_CR1_SSI | (0b10 << 3) | stm32.SPI_CR1_SSM) // Software Slave Management (NSS)
	stm32.SPI3.CR2.Set(stm32.SPI_CR2_FRXTH | (0b111 << 8))                                       // FIFO Threshold and 8bit size
	stm32.SPI3.CR1.SetBits(stm32.SPI_CR1_SPE)                                                    // Enable SPI

	println("RCC_APB3ENR=", stm32.RCC.APB3ENR.Get())
	println("RCC_CSR=", stm32.RCC.CSR.Get())
	println("PWR.SUBGHZSPICR=", stm32.PWR.SUBGHZSPICR.Get())
	println("EXTI.IMR2=", stm32.EXTI.IMR2.Get())
	println("PWR.CR3=", stm32.PWR.CR3.Get())
	println("PWR.SCR=", stm32.PWR.SCR.Get())
	println("SPI3.CR1=", stm32.SPI3.CR1.Get())
	println("SPI3.CR2=", stm32.SPI3.CR2.Get())

}

func SpiRx() uint8 {
	// Wait until Tx Buffer empty
	for !stm32.SPI3.SR.HasBits(stm32.SPI_SR_TXE) {
	}
	// Write Fake data to initiate a read
	stm32.SPI3.DR.Set(0xFF)
	// Wait for Rx Data
	for !stm32.SPI3.SR.HasBits(stm32.SPI_SR_RXNE) {
	}
	// Reat packet
	return uint8(stm32.SPI3.DR.Get() & 0xFF)

}

//SpiTx Transmit byte over SPI
func SpiTx(cmd uint8) {
	// Wait until Tx Buffer empty
	for !stm32.SPI3.SR.HasBits(stm32.SPI_SR_TXE) {
	}
	// Write to data register
	stm32.SPI3.DR.Set(uint32(cmd))
	// Wait for Rx Data
	for !stm32.SPI3.SR.HasBits(stm32.SPI_SR_RXNE) {
	}
	// Flush RX data
	stm32.SPI3.DR.Get()
}

// SpiExecCommand send a command to configure the peripheral
func SpiExecSetCommand(cmd uint8, buf []uint8) {
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS) // Select Slave
	SpiTx(cmd)                                                 // Send command
	for i := 0; i < len(buf); i++ {
		SpiTx(buf[i]) // Send options
	}
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS) // Disable Slave
}

// SpiExecCommand send a command to configure the peripheral
func SpiExecGetCommand(cmd uint8, size int) []uint8 {
	var buf []uint8
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS) // Select Slave
	SpiTx(cmd)                                                 // Send command
	SpiTx(0x00)                                                // Flush the status (1st byte)
	for i := 0; i < size; i++ {
		buf = append(buf, SpiRx()) // Read bytes
	}
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS) // Disable Slave
	return buf
}

//----------------------------------------------
//----------------------------------------------

func SetModulationParams(sf, bw, cr, ldro uint8) {
	var p [4]uint8
	p[0] = sf
	p[1] = bw
	p[2] = cr
	p[3] = ldro
}

func SetRfFrequency(f uint32) {
	var p [4]uint8
	p[0] = uint8((f >> 24) & 0xFF)
	p[1] = uint8((f >> 16) & 0xFF)
	p[2] = uint8((f >> 8) & 0xFF)
	p[3] = uint8((f >> 0) & 0xFF)
	SpiExecSetCommand(RADIO_SET_FR_FREQUENCY, p[:])
}

//----------------------------------------------
//----------------------------------------------

func xstatus(lora sx126x.Device) {
	dat := lora.GetStatus()
	cmdstatus := (dat & (0x7 << 4)) >> 4
	chipmode := (dat & (0x7 << 1)) >> 1
	println("cmdstatus:", cmdstatus, " chipmode:", chipmode)
}

func main() {
	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// UART0 (Console)
	machine.UART0.Configure(machine.UARTConfig{TX: machine.UART_TX_PIN, RX: machine.UART_TX_PIN, BaudRate: 9600})

	print("STM32WL Radio Init Example\n")

	// Prepare Lora
	lora := sx126x.New(machine.SPI0)

	initRadio()

	for {
		led.High()
		time.Sleep(time.Millisecond * 250)
		led.Low()

		stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)

		lora.Sleep()
		lora.Standby()

		xstatus(lora)

		lora.SetTx(0)

		xstatus(lora)

		time.Sleep(time.Millisecond * 1000)

	}
}
