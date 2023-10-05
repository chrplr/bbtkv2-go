// Drive a BlackBoxToolKit v2 to capture events
// Author: Christophe Pallier <christophe@pallier.org>

package main

import (
	"bufio"
	"fmt"
	"log"

	//"log"
	"time"
	//"strings"

	"go.bug.st/serial"
)

const (
	portAddress = "/dev/ttyACM0"
	baudrate    = 57400
	DEBUG       = true
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

	port, err := serial.Open(portAddress, mode)
	if err != nil {
		return nil, fmt.Errorf("Error while trying to open bbtkv2 at %s at %d bps: %w\n", portAddress, baudrate, err)
	}

	/* We might need to set timeout, writetimeout, xonxoff=False, rtscts=False,  dsrdtr=False, */

	box.port = port
	// box.port.SetDTR(false)
	// box.port.SetRTS(false)
	// box.port.SetReadTimeout(time.Second)

	box.reader = bufio.NewReader(port)

	return &box, nil
}

// Connect initiates a connection to the BBTKv2.
func (b bbtkv2) Connect() error {

	b.SendCommand("CONN")

	time.Sleep(10. * time.Millisecond)

	resp, err := b.ReadLine()
	if err != nil {
		return err
	}
	if resp != "BBTK;" {
		return fmt.Errorf("Connect: expected \"BBTK;\", got \"%v\"", resp)
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

func (b bbtkv2) SendCommand(cmd string) error {
	if err := b.port.ResetInputBuffer(); err != nil {
		return err
	}

	if err := b.port.ResetOutputBuffer(); err != nil {
		return err
	}

	if DEBUG {
		log.Printf("SendCommand: \"%v\"\n", cmd)
	}

	_, err := b.port.Write([]byte(cmd + "\r\n"))

	time.Sleep(50. * time.Millisecond)

	return err
}

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

// IsAlive sends an 'ECHO' command to the BBTKv2 and expects an 'ECHO' in return.
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

func (b bbtkv2) Flush() error {
	if err := b.SendCommand("FLUS"); err != nil {
		return err
	}
	time.Sleep(time.Second)
	return nil
}

func (b bbtkv2) GetFirmwareVersion() string {
	b.SendCommand("FIRM")
	resp, err := b.ReadLine()
	if err != nil {
		log.Printf("GetFirmWareVersion: %w", err)
	}
	return resp
}

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

func (b bbtkv2) ClearTimingData() {
	b.SendCommand("SPIE")

	response, err := b.ReadLine()
	if err != nil {
		log.Fatalf("ClearTimingData: %w", err)
	}
	if response != "FRMT;" {
		log.Printf("Warning: ClearTimingData expected \"FRMT;\", got \"%v\"", response)
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

func (b bbtkv2) DisplayInfoOnBBTK() {
	b.SendCommand("ABOU")
	time.Sleep(1. * time.Second)
}

func (b bbtkv2) SetDefaultsThresholds() {
	x := defaultThresholds
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

// Launches a digital data capture session.
// duration in seconds
func (b bbtkv2) CaptureEvents(duration int) string {
	var err error

	b.SendCommand("DSCM")
	time.Sleep(100 * time.Millisecond)
	b.SendCommand("TIML")
	b.SendCommand(fmt.Sprintf("%d", duration*1000000))
	err = b.SendCommand("RUDS")

	// n, err = b.port.Write([]byte(fmt.Sprintf("DSCM\r\nTIML\r\n%d\r\nRUDS\r\n", durationmicros)))
	if err != nil {
		log.Printf("CaptureEvents: %w", err)
	}

	//waitingDuration := time.Duration(duration-1) * time.Second
	//time.Sleep(waitingDuration)

	if DEBUG {
		fmt.Println("Waiting for data")
	}

	text := ""
	buff := make([]byte, 100)
	b.port.SetReadTimeout(time.Second)
	for {
		fmt.Print("+")
		n, err := b.port.Read(buff)
		if err != nil {
			log.Fatal(err)
			break
		}
		if n > 0 {
			fmt.Printf("%v", string(buff[:n]))
			text += string(buff[:n])
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

	fmt.Printf("Trying to connect to %v at %v...\n", portAddress, baudrate)
	b, err := NewBbtkv2(portAddress, baudrate)
	if err != nil {
		log.Fatalln(err)
	}
	defer b.Disconnect()

	b.SendBreak()
	time.Sleep(time.Second)

	if err = b.Connect(); err != nil {
		log.Fatalf("Connect returned: %w\n", err)
	}

	fmt.Printf("...Ok\n")

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

	b.DisplayInfoOnBBTK()

	fmt.Printf("Firmware version: %v\n", b.GetFirmwareVersion())

	fmt.Printf("Setting Smoothing mask to %+v\n", defaultSmoothingMask)
	if err = b.SetSmoothing(defaultSmoothingMask); err != nil {
		log.Printf("%w", err)
	}

	fmt.Printf("Setting thresholds: %+v\n", defaultThresholds)
	b.SetDefaultsThresholds()

	// fmt.Printf("Clearing Timing data... ")
	// b.ClearTimingData()
	// fmt.Println("Ok")

	time.Sleep(1 * time.Second)

	fmt.Println("Capturing events...")
	fmt.Println(b.CaptureEvents(10))

	if err = b.Disconnect(); err != nil {
		log.Println(err)
	}

}
