package main

import (
	"encoding/hex"

	"tinygo.org/x/drivers/lora"
	"tinygo.org/x/drivers/lora/lorawan/region"
)

// pktFwdTask is run as a gorouting when AT+TEST=PKTFWD
// it'll end when testMode is reset AT+TEST=STOP
func pktFwdTask() {
	println("# Packet Forward mode")
	r := region.EU868()
	ch := r.JoinAcceptChannel()

	radio.SetFrequency(ch.Frequency)
	radio.SetBandwidth(ch.Bandwidth)
	radio.SetCodingRate(ch.CodingRate)
	radio.SetSpreadingFactor(ch.SpreadingFactor)
	radio.SetPreambleLength(ch.PreambleLength)
	radio.SetTxPower(ch.TxPowerDBm)
	radio.SetHeaderType(lora.HeaderExplicit)
	radio.SetIqMode(lora.IQStandard)
	radio.SetCrc(true)

	cfg := radio.GetLoraConfig()
	print("FREQ:", cfg.Freq, " CR:", lora.CRToString(cfg.Cr))
	print(" BW:", lora.BWToString(cfg.Bw), " SF:", lora.SFToString(cfg.Sf))
	print(" CRC:", cfg.Crc, " IQ:", cfg.Iq, " HDR:", cfg.HeaderType, " PREAMBLE:", cfg.Preamble)
	println()

	for testMode == TEST_PKTFWD {
		data, err := radio.Rx(1000)
		if err == nil && len(data) > 0 {
			println("IN: size:", len(data), cfg.Freq, hex.EncodeToString(data))
		}
	}
}
