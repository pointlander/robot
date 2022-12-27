// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/color/palette"
	"image/draw"
	"image/gif"
	"io"
	"math/cmplx"
	"os"
	"os/exec"
	"runtime"
	"sort"
	"time"

	"github.com/mjibson/go-dsp/fft"
	"github.com/pointlander/gradient/tf32"
	"github.com/pointlander/occam"

	"github.com/blackjack/webcam"
	"github.com/nfnt/resize"
	"github.com/veandco/go-sdl2/sdl"
	"github.com/warthog618/gpiod"
	"github.com/warthog618/gpiod/device/rpi"
	"github.com/zergon321/reisen"
)

var joysticks = make(map[int]*sdl.Joystick)

type (
	// JoystickState is the state of a joystick
	JoystickState uint
	// Mode is the operating mode of the robot
	Mode uint
)

const (
	// JoystickStateNone is the default state of a joystick
	JoystickStateNone JoystickState = iota
	// JoystickStateUp is the state of a joystick when it is pushed up
	JoystickStateUp
	// JoystickStateDown is the state of a joystick when it is pushed down
	JoystickStateDown
)

const (
	// ModeManual
	ModeManual Mode = iota
	// ModeAuto
	ModeAuto
)

const (
	// Width is the width of the fft
	Width = 24
	// Height is the height of the fft
	Height = 24
	// States is the number of states
	States = 3
	// Memory is the sive of memory per state
	Memory = 3
)

var (
	// FlagPicture is the flag for taking a picture
	FlagPicture = flag.Bool("picture", false, "take a picture")
)

// String returns a string representation of the JoystickState
func (j JoystickState) String() string {
	switch j {
	case JoystickStateUp:
		return "up"
	case JoystickStateDown:
		return "down"
	default:
		return "none"
	}
}

type FrameSizes []webcam.FrameSize

func (slice FrameSizes) Len() int {
	return len(slice)
}

//For sorting purposes
func (slice FrameSizes) Less(i, j int) bool {
	ls := slice[i].MaxWidth * slice[i].MaxHeight
	rs := slice[j].MaxWidth * slice[j].MaxHeight
	return ls < rs
}

//For sorting purposes
func (slice FrameSizes) Swap(i, j int) {
	slice[i], slice[j] = slice[j], slice[i]
}

// Frame is a video frame
type Frame struct {
	Frame image.Image
	DCT   [][]float64
}

// StreamCamera is a camera that is from a stream
type StreamCamera struct {
	Stream bool
	Images chan Frame
}

// NewStreamCamera creates a new streaming camera
func NewStreamCamera() *StreamCamera {
	return &StreamCamera{
		Stream: true,
		Images: make(chan Frame, 8),
	}
}

// Start starts streaming
func (sc *StreamCamera) Start() {
	command := exec.Command("libcamera-vid", "-t", "0", "-o", "-")
	input, err := command.StdoutPipe()
	if err != nil {
		panic(err)
	}
	err = command.Start()
	if err != nil {
		panic(err)
	}
	output, err := os.OpenFile("center", os.O_RDWR, 0600)
	if err != nil {
		panic(err)
	}
	go io.Copy(output, input)
	defer close(sc.Images)

	media, err := reisen.NewMedia("center")
	if err != nil {
		panic(err)
	}
	defer media.Close()
	err = media.OpenDecode()
	if err != nil {
		panic(err)
	}

	for sc.Stream {
		var pkt *reisen.Packet
		pkt, gotPacket, err := media.ReadPacket()
		if err != nil {
			panic(err)
		}

		if !gotPacket {
			break
		}

		switch pkt.Type() {
		case reisen.StreamVideo:
			s := media.Streams()[pkt.StreamIndex()].(*reisen.VideoStream)
			if !s.Opened() {
				err = s.Open()
				if err != nil {
					panic(err)
				}
			}

			videoFrame, gotFrame, err := s.ReadVideoFrame()
			if err != nil {
				panic(err)
			}

			if !gotFrame {
				break
			}

			if videoFrame == nil {
				continue
			}

			tiny := resize.Resize(Width, Height, videoFrame.Image(), resize.Lanczos3)
			b := tiny.Bounds()
			gray := image.NewGray(b)
			for y := 0; y < b.Max.Y; y++ {
				for x := 0; x < b.Max.X; x++ {
					original := tiny.At(x, y)
					pixel := color.GrayModel.Convert(original)
					gray.Set(x, y, pixel)
				}
			}
			width, height := b.Max.X, b.Max.Y
			pixels := make([][]float64, height)
			for j := range pixels {
				pix := make([]float64, width)
				for i := range pix {
					pix[i] = float64(gray.At(i, j).(color.Gray).Y) / 255
				}
				pixels[j] = pix
			}
			output := fft.FFT2Real(pixels)
			for j, pix := range pixels {
				for i := range pix {
					pix[i] = cmplx.Abs(output[j][i]) / float64(width*height)
				}
			}
			select {
			case sc.Images <- Frame{
				Frame: videoFrame.Image(),
				DCT:   pixels,
			}:
			default:
				fmt.Println("drop")
			}
		case reisen.StreamAudio:
			s := media.Streams()[pkt.StreamIndex()].(*reisen.AudioStream)

			if !s.Opened() {
				err = s.Open()
				if err != nil {
					panic(err)
				}
			}

			audioFrame, gotFrame, err := s.ReadAudioFrame()
			if err != nil {
				panic(err)
			}

			if !gotFrame {
				break
			}

			if audioFrame == nil {
				continue
			}
		}
	}

	for _, stream := range media.Streams() {
		err = stream.Close()
		if err != nil {
			panic(err)
		}
	}

	err = media.CloseDecode()
	if err != nil {
		panic(err)
	}

	err = command.Wait()
	if err != nil {
		panic(err)
	}
}

// V4LCamera is a camera that is from a v4l device
type V4LCamera struct {
	Stream bool
	Images chan Frame
}

// NewV4LCamera creates a new v4l camera
func NewV4LCamera() *V4LCamera {
	return &V4LCamera{
		Stream: true,
		Images: make(chan Frame, 8),
	}
}

// Start starts streaming
func (vc *V4LCamera) Start(device string) {
	runtime.LockOSThread()
	fmt.Println(device)
	camera, err := webcam.Open(device)
	if err != nil {
		panic(err)
	}
	defer camera.Close()

	format_desc := camera.GetSupportedFormats()
	var formats []webcam.PixelFormat
	for f := range format_desc {
		formats = append(formats, f)
	}
	sort.Slice(formats, func(i, j int) bool {
		return format_desc[formats[i]] < format_desc[formats[j]]
	})
	println("Available formats: ")
	for i, value := range formats {
		fmt.Printf("[%d] %s\n", i+1, format_desc[value])
	}
	format := formats[1]

	fmt.Printf("Supported frame sizes for format %s\n", format_desc[format])
	frames := FrameSizes(camera.GetSupportedFrameSizes(format))
	sort.Sort(frames)
	for i, value := range frames {
		fmt.Printf("[%d] %s\n", i+1, value.GetString())
	}
	size := frames[0]

	f, w, h, err := camera.SetImageFormat(format, uint32(size.MaxWidth), uint32(size.MaxHeight))
	if err != nil {
		panic(err)
	} else {
		fmt.Printf("Resulting image format: %s (%dx%d)\n", format_desc[f], w, h)
	}

	err = camera.StartStreaming()
	if err != nil {
		panic(err)
	}
	defer camera.StopStreaming()

	var cp []byte
	err = camera.WaitForFrame(5)

	switch err.(type) {
	case nil:
	case *webcam.Timeout:
		fmt.Fprint(os.Stderr, err.Error())
	default:
		panic(err.Error())
	}

	for vc.Stream {
		frame, err := camera.ReadFrame()
		if err != nil {
			time.Sleep(time.Second)
			fmt.Println(device, err)
			continue
		} else {
			fmt.Println(device)
		}
		if len(frame) != 0 {
			if len(cp) < len(frame) {
				cp = make([]byte, len(frame))
			}
			copy(cp, frame)
			fmt.Printf("Frame: %d bytes\n", len(cp))
			yuyv := image.NewYCbCr(image.Rect(0, 0, int(w), int(h)), image.YCbCrSubsampleRatio422)
			for i := range yuyv.Cb {
				ii := i * 4
				yuyv.Y[i*2] = cp[ii]
				yuyv.Y[i*2+1] = cp[ii+2]
				yuyv.Cb[i] = cp[ii+1]
				yuyv.Cr[i] = cp[ii+3]

			}
			tiny := resize.Resize(Width, Height, yuyv, resize.Lanczos3)
			b := tiny.Bounds()
			gray := image.NewGray(b)
			for y := 0; y < b.Max.Y; y++ {
				for x := 0; x < b.Max.X; x++ {
					original := tiny.At(x, y)
					pixel := color.GrayModel.Convert(original)
					gray.Set(x, y, pixel)
				}
			}
			width, height := b.Max.X, b.Max.Y
			pixels := make([][]float64, height)
			for j := range pixels {
				pix := make([]float64, width)
				for i := range pix {
					pix[i] = float64(gray.At(i, j).(color.Gray).Y) / 255
				}
				pixels[j] = pix
			}
			output := fft.FFT2Real(pixels)
			for j, pix := range pixels {
				for i := range pix {
					pix[i] = cmplx.Abs(output[j][i]) / float64(width*height)
				}
			}
			select {
			case vc.Images <- Frame{
				Frame: yuyv,
				DCT:   pixels,
			}:
			default:
				fmt.Println("drop")
			}
		}
	}
}

func main() {
	flag.Parse()

	if *FlagPicture {
		//picture()
		stream := NewStreamCamera()
		left := NewV4LCamera()
		right := NewV4LCamera()
		go stream.Start()
		go left.Start("/dev/videol")
		go right.Start("/dev/videor")
		i, j, k := 0, 0, 0
		var c, l, r []*image.Paletted
		for i < 32 || j < 32 || k < 32 {
			select {
			case img := <-stream.Images:
				opts := gif.Options{
					NumColors: 256,
					Drawer:    draw.FloydSteinberg,
				}
				bounds := img.Frame.Bounds()
				paletted := image.NewPaletted(bounds, palette.Plan9[:opts.NumColors])
				if opts.Quantizer != nil {
					paletted.Palette = opts.Quantizer.Quantize(make(color.Palette, 0, opts.NumColors), img.Frame)
				}
				opts.Drawer.Draw(paletted, bounds, img.Frame, image.Point{})
				c = append(c, paletted)
				fmt.Println("center", i)
				i++
			case img := <-left.Images:
				opts := gif.Options{
					NumColors: 256,
					Drawer:    draw.FloydSteinberg,
				}
				bounds := img.Frame.Bounds()
				paletted := image.NewPaletted(bounds, palette.Plan9[:opts.NumColors])
				if opts.Quantizer != nil {
					paletted.Palette = opts.Quantizer.Quantize(make(color.Palette, 0, opts.NumColors), img.Frame)
				}
				opts.Drawer.Draw(paletted, bounds, img.Frame, image.Point{})
				l = append(l, paletted)
				fmt.Println("left", j)
				j++
			case img := <-right.Images:
				opts := gif.Options{
					NumColors: 256,
					Drawer:    draw.FloydSteinberg,
				}
				bounds := img.Frame.Bounds()
				paletted := image.NewPaletted(bounds, palette.Plan9[:opts.NumColors])
				if opts.Quantizer != nil {
					paletted.Palette = opts.Quantizer.Quantize(make(color.Palette, 0, opts.NumColors), img.Frame)
				}
				opts.Drawer.Draw(paletted, bounds, img.Frame, image.Point{})
				r = append(r, paletted)
				fmt.Println("right", k)
				k++
			}
		}
		stream.Stream = false
		left.Stream = false
		right.Stream = false
		process := func(name string, images []*image.Paletted) {
			animation := &gif.GIF{}
			for _, paletted := range images {
				animation.Image = append(animation.Image, paletted)
				animation.Delay = append(animation.Delay, 0)
			}

			f, _ := os.OpenFile(name, os.O_WRONLY|os.O_CREATE, 0600)
			defer f.Close()
			gif.EncodeAll(f, animation)
		}
		process("center.gif", c)
		process("left.gif", l)
		process("right.gif", r)
		return
	}

	var event sdl.Event
	var running bool
	sdl.Init(sdl.INIT_JOYSTICK)
	defer sdl.Quit()
	sdl.JoystickEventState(sdl.ENABLE)
	running = true
	var axis [5]int16
	joystickLeft := JoystickStateNone
	joystickRight := JoystickStateNone
	var speed int16
	var mode Mode

	in1, err := gpiod.RequestLine("gpiochip0", rpi.GPIO20, gpiod.AsOutput(0))
	if err != nil {
		panic(err)
	}
	in2, err := gpiod.RequestLine("gpiochip0", rpi.GPIO21, gpiod.AsOutput(0))
	if err != nil {
		panic(err)
	}
	in3, err := gpiod.RequestLine("gpiochip0", rpi.GPIO19, gpiod.AsOutput(0))
	if err != nil {
		panic(err)
	}
	in4, err := gpiod.RequestLine("gpiochip0", rpi.GPIO26, gpiod.AsOutput(0))
	if err != nil {
		panic(err)
	}
	ena, err := gpiod.RequestLine("gpiochip0", rpi.GPIO16, gpiod.AsOutput(0))
	if err != nil {
		panic(err)
	}
	enb, err := gpiod.RequestLine("gpiochip0", rpi.GPIO13, gpiod.AsOutput(0))
	if err != nil {
		panic(err)
	}
	servoUpDown, err := gpiod.RequestLine("gpiochip0", rpi.GPIO9, gpiod.AsOutput(0))
	if err != nil {
		panic(err)
	}
	servoLeftRight, err := gpiod.RequestLine("gpiochip0", rpi.GPIO11, gpiod.AsOutput(0))
	if err != nil {
		panic(err)
	}
	pwm := 50
	t := time.Tick(5 * time.Microsecond)
	go func() {
		counter, state := 0, 0
		for {
			<-t
			counter++
			if counter%100 > pwm {
				state = 1
			} else {
				state = 0
			}
			ena.SetValue(state)
			enb.SetValue(state)
		}
	}()

	update := func() {
		switch joystickRight {
		case JoystickStateUp:
			fmt.Println("right up")
			in3.SetValue(1)
			in4.SetValue(0)
		case JoystickStateDown:
			fmt.Println("right down")
			in3.SetValue(0)
			in4.SetValue(1)
		default:
			in3.SetValue(0)
			in4.SetValue(0)
		}
		switch joystickLeft {
		case JoystickStateUp:
			fmt.Println("left up")
			in1.SetValue(1)
			in2.SetValue(0)
		case JoystickStateDown:
			fmt.Println("left down")
			in1.SetValue(0)
			in2.SetValue(1)
		default:
			in1.SetValue(0)
			in2.SetValue(0)
		}
	}

	pwmUpDownServo := 1500
	pwmLeftRightServo := 1500

	stream := NewStreamCamera()
	left := NewV4LCamera()
	right := NewV4LCamera()
	go stream.Start()
	go left.Start("/dev/videol")
	go right.Start("/dev/videor")

	go func() {
		net, s := occam.NewNetwork(Width*Height, 3), 0
		var state [States]int
		for running {
			var line [][]float64
			var index int
			select {
			case frame := <-stream.Images:
				index = 0
				line = frame.DCT
			case frame := <-left.Images:
				index = 1
				line = frame.DCT
			case frame := <-right.Images:
				index = 2
				line = frame.DCT
			}
			i := 0
			for y := 0; y < Height; y++ {
				for x := 0; x < Width; x++ {
					net.Point.X[index*Width*Height+i] = float32(line[y][x])
					i++
				}
			}

			max, index := float32(0.0), 0
			for i := 0; i < 3; i++ {
				for i, value := range net.Point.X[i*Width*Height : (i+1)*Width*Height] {
					net.Input.X[i] = float32(value)
				}
				net.Cost(func(a *tf32.V) bool {
					if a.X[0] > max {
						max = a.X[0]
						index = i
					}
					return true
				})
			}
			state[s] = index
			s = (s + 1) % States
			var histogram [States]int
			for _, value := range state {
				histogram[value]++
			}
			index = 0
			{
				max := 0
				for i, value := range histogram {
					if value > max {
						max = value
						index = i
					}
				}
			}
			if mode == ModeAuto {
				switch index {
				case 0:
					fmt.Println("Forward")
					joystickLeft = JoystickStateUp
					joystickRight = JoystickStateUp
				case 1:
					fmt.Println("Left")
					joystickLeft = JoystickStateDown
					joystickRight = JoystickStateUp
				case 2:
					fmt.Println("Right")
					joystickLeft = JoystickStateUp
					joystickRight = JoystickStateDown
				}
				update()
			}
		}
	}()

	for running {
		for event = sdl.PollEvent(); event != nil; event = sdl.PollEvent() {
			switch t := event.(type) {
			case *sdl.QuitEvent:
				running = false
			case *sdl.JoyAxisEvent:
				value := int16(t.Value)
				axis[t.Axis] = value
				if t.Axis == 3 || t.Axis == 4 {
					if mode == ModeManual {
						if axis[3] < 20000 && axis[3] > -20000 {
							if axis[4] < -32000 {
								joystickRight = JoystickStateUp
							} else if axis[4] > 32000 {
								joystickRight = JoystickStateDown
							} else {
								joystickRight = JoystickStateNone
							}
						} else {
							joystickRight = JoystickStateNone
						}
					}
					//fmt.Printf("right [%d ms] Which: %v \t%d %d\n",
					//		t.Timestamp, t.Which, axis[3], axis[4])
				} else if t.Axis == 0 || t.Axis == 1 {
					if mode == ModeManual {
						if axis[0] < 20000 && axis[0] > -20000 {
							if axis[1] < -32000 {
								joystickLeft = JoystickStateUp
							} else if axis[1] > 32000 {
								joystickLeft = JoystickStateDown
							} else {
								joystickLeft = JoystickStateNone
							}
						} else {
							joystickLeft = JoystickStateNone
						}
					}
					//fmt.Printf("left [%d ms] Which: %v \t%d %d\n",
					//t.Timestamp, t.Which, axis[0], axis[1])
				} else if t.Axis == 2 {
					//fmt.Printf("2 axis [%d ms] Which: %v \t%x\n",
					//	t.Timestamp, t.Which, value)
					speed = axis[2]
					pwm = int(100 * (float64(speed) + 32768) / 65535)
					fmt.Printf("speed %d pwm %d\n", speed, pwm)
				}

				update()
			case *sdl.JoyBallEvent:
				fmt.Printf("[%d ms] Ball:%d\txrel:%d\tyrel:%d\n",
					t.Timestamp, t.Ball, t.XRel, t.YRel)
			case *sdl.JoyButtonEvent:
				fmt.Printf("[%d ms] Button:%d\tstate:%d\n",
					t.Timestamp, t.Button, t.State)
				if t.Button == 0 && t.State == 1 {
					switch mode {
					case ModeManual:
						mode = ModeAuto
					case ModeAuto:
						mode = ModeManual
						joystickLeft = JoystickStateNone
						joystickRight = JoystickStateNone
						update()
					}
				} else if t.Button == 1 && t.State == 1 {
					pwm = (pwm + 25) % 100
				}
			case *sdl.JoyHatEvent:
				fmt.Printf("[%d ms] Hat:%d\tvalue:%d\n",
					t.Timestamp, t.Hat, t.Value)
				if t.Value == 1 {
					// up
					if pwmUpDownServo < 2500 {
						pwmUpDownServo += 100
						servoUpDown.SetValue(1)
						done := time.After(time.Duration(pwmUpDownServo) * time.Microsecond)
						go func() {
							<-done
							servoUpDown.SetValue(0)
						}()
					}
				} else if t.Value == 4 {
					// down
					if pwmUpDownServo > 100 {
						pwmUpDownServo -= 100
						servoUpDown.SetValue(1)
						done := time.After(time.Duration(pwmUpDownServo) * time.Microsecond)
						go func() {
							<-done
							servoUpDown.SetValue(0)
						}()
					}
				} else if t.Value == 8 {
					// left
					if pwmLeftRightServo < 2500 {
						pwmLeftRightServo += 100
						servoLeftRight.SetValue(1)
						done := time.After(time.Duration(pwmLeftRightServo) * time.Microsecond)
						go func() {
							<-done
							servoLeftRight.SetValue(0)
						}()
					}
				} else if t.Value == 2 {
					// right
					if pwmLeftRightServo > 100 {
						pwmLeftRightServo -= 100
						servoLeftRight.SetValue(1)
						done := time.After(time.Duration(pwmLeftRightServo) * time.Microsecond)
						go func() {
							<-done
							servoLeftRight.SetValue(0)
						}()
					}
				}
			case *sdl.JoyDeviceAddedEvent:
				fmt.Println(t.Which)
				joysticks[int(t.Which)] = sdl.JoystickOpen(int(t.Which))
				if joysticks[int(t.Which)] != nil {
					fmt.Printf("Joystick %d connected\n", t.Which)
				}
			case *sdl.JoyDeviceRemovedEvent:
				if joystick := joysticks[int(t.Which)]; joystick != nil {
					joystick.Close()
				}
				fmt.Printf("Joystick %d disconnected\n", t.Which)
			default:
				fmt.Printf("Unknown event\n")
			}
		}

		sdl.Delay(16)
	}
}
