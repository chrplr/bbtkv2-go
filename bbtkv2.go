// Drive a BlackBoxToolKit v2 to capture events
// Author: Christophe Pallier <christophe@pallier.org>
// LICENSE: GPL-3.0

package main

import (
	"bufio"
	"flag"
	"fmt"
	"log"

	//"log"
	"strings"
	"time"

	"go.bug.st/serial"
)

// default parameters
var (
	portAddress = "/dev/ttyACM0"
	baudrate    = 57600
	duration    = 30
	DEBUG       = false
)

type bbtkv2 struct {
	port   serial.Port
	reader *bufio.Reader
}

type thresholds struct {
	Mic1     uint8
	Mic2     uint8
	Sounder1 uint8
	Sounder2 uint8
	Opto1    uint8
	Opto2    uint8
	Opto3    uint8
	Opto4    uint8
}

var defaultThresholds = thresholds{
	Mic1:     0,
	Mic2:     0,
	Sounder1: 63,
	Sounder2: 63,
	Opto1:    110,
	Opto2:    110,
	Opto3:    110,
	Opto4:    110,
}

type smoothingMask struct {
	Mic1  bool
	Mic2  bool
	Opto4 bool
	Opto3 bool
	Opto2 bool
	Opto1 bool
}

var defaultSmoothingMask = smoothingMask{
	Mic1:  true,
	Mic2:  true,
	Opto4: false,
	Opto3: false,
	Opto2: false,
	Opto1: false,
}

func NewBbtkv2(portAddress string, baudrate int) (*bbtkv2, error) {
	var box bbtkv2

	mode := &serial.Mode{
		BaudRate: baudrate,
		Parity:   serial.NoParity,
		DataBits: 8,
		StopBits: serial.OneStopBit,
	}

	if DEBUG {
		fmt.Printf("Trying to connect to %v at %dbps...", portAddress, baudrate)
	}
	port, err := serial.Open(portAddress, mode)
	if err != nil {
		return nil, fmt.Errorf("Error while trying to open bbtkv2 at %s at %d bps: %w\n", portAddress, baudrate, err)
	}

	if DEBUG {
		fmt.Println("Success!")
	}

	port.SetReadTimeout(time.Second)
	// port.SetDTR(false)
	// port.SetRTS(false)

	box.port = port
	box.reader = bufio.NewReader(port)

	return &box, nil
}

// Connect initiates a connection to the BBTKv2.
func (b bbtkv2) Connect() error {

	if DEBUG {
		fmt.Printf("Trying to connect to bbtkv2...")
	}

	b.SendCommand("CONN")

	time.Sleep(10. * time.Millisecond)

	resp, err := b.ReadLine()
	if err != nil {
		return err
	}
	if resp != "BBTK;" {
		return fmt.Errorf("Connect: expected \"BBTK;\", got \"%v\"", resp)
	}

	if DEBUG {
		fmt.Println("Success!")
	}
	return nil
}

func (b bbtkv2) Disconnect() error {
	b.SendBreak()
	return b.port.Close()
}

func (b bbtkv2) SendBreak() {
	if DEBUG {
		log.Println("Sending serial break.")
	}
	b.port.Break(10. * time.Millisecond)
	time.Sleep(time.Second)
}

func (b bbtkv2) ResetSerialBuffers() error {
	if err := b.port.ResetInputBuffer(); err != nil {
		return err
	}

	return b.port.ResetOutputBuffer()
}

// SendCommand adds CRLF to cmd and send it to the BBTK
func (b bbtkv2) SendCommand(cmd string) error {

	if DEBUG {
		log.Printf("SendCommand: \"%v\"\n", cmd)
	}

	_, err := b.port.Write([]byte(cmd + "\r\n"))

	time.Sleep(50. * time.Millisecond)

	return err
}

// ReadLine returns the next line output by the BBTK
func (b bbtkv2) ReadLine() (string, error) {
	var s string
	var err error
	if s, err = b.reader.ReadString('\n'); err != nil {
		return "", fmt.Errorf("Readline: %w", err)
	}
	if DEBUG {
		log.Printf("Readline: got \"%s\"\n", s[:len(s)-1])
	}
	return s[:len(s)-1], err
}

/*
	 func (b bbtkv2) ReadResponse() (string, error) {
		self.global_data = ""
		while (self.bbtk.is_open and ((time.time() - last_time) < timeout)):
			if (self.bbtk.in_waiting > 0):
				chars = self.bbtk.read(self.bbtk.in_waiting).decode('ascii')
				data_str += chars
				self.global_data += chars
				if self.debug:
					print(chars, end='')
				last_time = time.time()
			time.sleep(0.05)

		return data_str
	}
*/

// IsAlive sends an 'ECHO' command to the BBTKv2 and expects 'ECHO' in return.
// This permits to check that the BBTKv2 is up and running.
func (b bbtkv2) IsAlive() (bool, error) {

	if err := b.SendCommand("ECHO"); err != nil {
		return false, fmt.Errorf("IsAlive: %w", err)
	} else {
		resp, err := b.ReadLine()
		if err != nil {
			return false, fmt.Errorf("IsAlive: %w", err)
		}

		if resp != "ECHO" {
			return true, fmt.Errorf("IsAlive: Expected \"ECHO\", Got \"%v\"", resp)
		} else {
			return true, nil
		}
	}

}

// SetSmoothing on Opto and Mic sensors.
// When smoothing is 'off', the BBTK will detect *all* leading edges, e.g.
// each refresh on a CRT.
// When smoothing is 'on', you need to subtract 20ms from offset times.
func (b bbtkv2) SetSmoothing(mask smoothingMask) error {
	if err := b.SendCommand("SMOO"); err != nil {
		return fmt.Errorf("SetSmoothing: %w", err)
	}

	strMask := ""

	if mask.Mic1 {
		strMask += "1"
	} else {
		strMask += "0"
	}

	if mask.Mic2 {
		strMask += "1"
	} else {
		strMask += "0"
	}

	if mask.Opto4 {
		strMask += "1"
	} else {
		strMask += "0"
	}

	if mask.Opto3 {
		strMask += "1"
	} else {
		strMask += "0"
	}

	if mask.Opto2 {
		strMask += "1"
	} else {
		strMask += "0"
	}

	if mask.Opto1 {
		strMask += "1"
	} else {
		strMask += "0"
	}

	strMask += "11"

	err := b.SendCommand(strMask)
	if err != nil {
		return fmt.Errorf("SetSmoothing: %w", err)
	}
	return nil
}

// FLUS command attempts to clear the USB output buffer.
// If this fails you may need to send a Serial Break with SendBreak().
func (b bbtkv2) Flush() error {
	if err := b.SendCommand("FLUS"); err != nil {
		return err
	}
	time.Sleep(time.Second)
	return nil
}

// Retrieves the version of the BBTK firmware
// currently running in the ARM chip.
func (b bbtkv2) GetFirmwareVersion() string {
	b.SendCommand("FIRM")
	resp, err := b.ReadLine()
	if err != nil {
		log.Printf("GetFirmWareVersion: %w", err)
	}
	return resp
}

// AdjustThresholds launches the procedure to manually set up the thresholds on the BBTK
func (b bbtkv2) AdjustThresholds() {
	b.SendCommand("AJPV")
	response, _ := b.ReadLine()
	for response != "Done;" {
		if DEBUG {
			log.Printf("Adjusting Threshold: expecting \"Done;\", got \"%v\"", response)
		}
		time.Sleep(100. * time.Millisecond)
		response, _ = b.ReadLine()
	}
}

// ClearTimingData either formats the whole of the BBTK's internal
// RAM (on first power up or after a reset) or erases
// only previously used sectors.
func (b bbtkv2) ClearTimingData() {
	b.SendCommand("SPIE")

	response, err := b.ReadLine()
	if err != nil {
		log.Fatalf("ClearTimingData: %w", err)
	}
	if response != "FRMT;" && response != "ESEC;" {
		log.Printf("Warning: ClearTimingData expected \"FRMT;\" or \"ESEC;\", got \"%v\"", response)
	}

	response, err = b.ReadLine()
	if err != nil {
		log.Fatalf("ClearTimingData: %w", err)
	}

	for response != "DONE;" {
		if DEBUG {
			log.Printf("Warning: ClearTimingData expected \"DONE;\", got \"%v\"", response)
		}

		time.Sleep(100. * time.Millisecond)
		response, err = b.ReadLine()
		if err != nil {
			log.Fatalf("ClearTimingData: %w", err)
		}
	}

	time.Sleep(time.Second)
}

// DisplayInfoOnBBTK causes the BBTK to display a copyright notice
// and release date of the firmware it is running on its LCD screen.
func (b bbtkv2) DisplayInfoOnBBTK() {
	b.SendCommand("ABOU")
	time.Sleep(1. * time.Second)
}

// Sets the sensor activation thresholds for the eight
// adjustable lines, i.e. Mic activation threshold,
// Sounder volume (amplitude) and Opto luminance
// activation threshold. Activation thresholds range
// from 0-127.
func (b bbtkv2) SetThresholds(x thresholds) {
	b.SendCommand("SEPV")
	b.SendCommand(fmt.Sprintf("%d", x.Mic1))
	b.SendCommand(fmt.Sprintf("%d", x.Mic2))
	b.SendCommand(fmt.Sprintf("%d", x.Sounder1))
	b.SendCommand(fmt.Sprintf("%d", x.Sounder2))
	b.SendCommand(fmt.Sprintf("%d", x.Opto1))
	b.SendCommand(fmt.Sprintf("%d", x.Opto2))
	b.SendCommand(fmt.Sprintf("%d", x.Opto3))
	b.SendCommand(fmt.Sprintf("%d", x.Opto4))

	time.Sleep(1 * time.Second)
}

func (b bbtkv2) SetDefaultsThresholds() {
	b.SetThresholds(defaultThresholds)
}

// Launches a digital data capture session.
// duration in seconds
func (b bbtkv2) CaptureEvents(duration int) string {
	var err error
	time.Sleep(time.Second)
	err = b.SendCommand("DSCM")
	if err != nil {
		log.Printf("CaptureEvents: DSCM %w", err)
	}

	time.Sleep(time.Second)
	err = b.SendCommand("TIML")
	if err != nil {
		log.Printf("CaptureEvents: TIML %w", err)
	}

	time.Sleep(time.Second)
	err = b.SendCommand(fmt.Sprintf("%d", duration*1000000))
	if err != nil {
		log.Printf("CaptureEvents: %w", err)
	}

	time.Sleep(time.Second)
	time.Sleep(500 * time.Millisecond)
	err = b.SendCommand("RUDS")
	if err != nil {
		log.Printf("CaptureEvents: RUDS %w", err)
	}

	waitingDuration := time.Duration(duration-1) * time.Second
	time.Sleep(waitingDuration)

	if DEBUG {
		fmt.Println("Waiting for data...")
	}

	text := ""
	buff := make([]byte, 100)
	//b.port.SetReadTimeout(time.Second)
	for {

		n, err := b.port.Read(buff)
		if err != nil {
			log.Fatal(err)
			break
		}
		if n > 0 {
			fmt.Printf("%v", string(buff[:n]))
			text += string(buff[:n])
		}
		if strings.Contains(string(buff), "EDAT") {
			break
		}
	}

	/* 	output := ""
	   	b.port.SetReadTimeout(2 * time.Second)
	   	inp, err := b.ReadLine() // we should get "SDAT;"
	   	if err != nil {
	   		log.Print("shit!")
	   	}

	   	for inp != "EDAT;" {
	   		output += b.port.ResetInputBuffer().Error()
	   		inp, err = b.ReadLine()
	   		if err != nil {
	   			log.Print("shit!")
	   		}

	   	}
	*/
	return text

	/* text = self.get_response(5)
	   if output_file is not None:
	       with open(output_file, 'wt') as f:
	           f.write(text)

	   lines = text.split('\n')
	   assert lines[1]=='SDAT;'
	   nevents = lines[2]
	   capture_time = lines[3]
	   nsamples = lines[4]
	   events = lines[5:-2]
	   print("Total number of events=", len(events))
	   return text
	*/
}

func main() {

	portPtr := flag.String("p", portAddress, "device (serial port name)")
	speedPtr := flag.Int("b", baudrate, "baudrate (speed in bps)")
	durationPtr := flag.Int("d", duration, "duration of capture (in s)")
	debugPtr := flag.Bool("v", DEBUG, "verbose (set to 'true' to debug)")

	flag.Parse()

	DEBUG = *debugPtr

	b, err := NewBbtkv2(*portPtr, *speedPtr)
	if err != nil {
		log.Fatalln(err)
	}
	defer b.Disconnect()

	b.SendBreak()
	time.Sleep(time.Second)

	err = b.ResetSerialBuffers()
	if err != nil {
		log.Printf("ResetSerialIOBuff %v\n", err)
	}

	if err = b.Connect(); err != nil {
		log.Fatalf("Connect returned: %w\n", err)
	}
	time.Sleep(time.Second)

	err = b.ResetSerialBuffers()
	if err != nil {
		log.Printf("ResetSerialIOBuff %v\n", err)
	}

	var alive bool
	if alive, err = b.IsAlive(); err != nil {
		log.Println(err)
	} else {
		if alive {
			fmt.Println("BBTKv2 is alive")
		} else {
			fmt.Println("BBTKv2 not responding to ECHO")
		}
	}
	time.Sleep(time.Second)

	// b.DisplayInfoOnBBTK()

	//fmt.Printf("Firmware version: %v\n", b.GetFirmwareVersion())

	fmt.Printf("Setting Smoothing mask to %+v\n", defaultSmoothingMask)
	if err = b.SetSmoothing(defaultSmoothingMask); err != nil {
		log.Printf("%w", err)
	}
	time.Sleep(time.Second)

	fmt.Printf("Setting thresholds: %+v\n", defaultThresholds)
	b.SetDefaultsThresholds()

	time.Sleep(time.Second)
	fmt.Printf("Clearing Timing data... ")
	b.ClearTimingData()
	fmt.Println("Ok")

	time.Sleep(1 * time.Second)
	fmt.Println(b.CaptureEvents(*durationPtr))

	if err = b.Disconnect(); err != nil {
		log.Println(err)
	}

}
