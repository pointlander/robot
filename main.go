// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

//go:generate protoc --go_out=. protocol.proto

package main

import (
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/color/palette"
	"image/draw"
	"image/gif"
	"math"
	"math/rand"
	"os"
	"time"

	. "github.com/pointlander/matrix"

	"github.com/veandco/go-sdl2/sdl"
	"github.com/warthog618/gpiod"
	"github.com/warthog618/gpiod/device/rpi"
)

var joysticks = make(map[int]*sdl.Joystick)

type (
	// JoystickState is the state of a joystick
	JoystickState uint
	// Mode is the operating mode of the robot
	Mode uint
	// Camera is a camera
	TypeCamera uint
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
	// CameraCenter is the center camera
	TypeCameraCenter TypeCamera = iota
	// CameraLeft is the left camera
	TypeCameraLeft
	// CameraRight is the right camera
	TypeCameraRight
	// CameraNone is no camera
	TypeCameraNone
)

const (
	// Window is the window size
	Window = 128
	// Rate is the learning rate
	Rate = .3
	// Outputs is the number of outputs
	Outputs = 64
	// Nets is the number of camera nets
	Nets = 16
	// Pixels is the number of pixels to sample
	Pixels = 128
)

// Coord is a coordinate
type Coord struct {
	X int
	Y int
}

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

// Frame is a video frame
type Frame struct {
	Frame image.Image
	DCT   [][]float64
	Query Matrix
	Key   Matrix
	Value Matrix
}

// FrameProcessor is a frame processor
type FrameProcessor struct {
	Seed   int64
	Net    Net
	Nets   []Net
	Coords [][]Coord
	Input  chan Input
}

// NewFrameProcessor creates a new frame processor
func NewFrameProcessor(seed int64) *FrameProcessor {
	nets := make([]Net, Nets)
	for n := range nets {
		nets[n] = NewNet(seed+1+int64(n), Window, 3*Pixels, 8)
	}
	return &FrameProcessor{
		Seed:  seed,
		Net:   NewNet(seed, Window, Nets*8, Outputs),
		Nets:  nets,
		Input: make(chan Input, 1),
	}
}

// Convert converts an image into an input
func Convert(source uint32, img image.Image) Input {
	b := img.Bounds()
	width, height := b.Max.X, b.Max.Y
	pixels := make([]uint32, 0, 3*width*height)
	for i := 0; i < height; i++ {
		for j := 0; j < width; j++ {
			pixel := img.At(i, j)
			r, g, b, _ := pixel.RGBA()
			y, cb, cr := color.RGBToYCbCr(uint8(r>>8), uint8(g>>8), uint8(b>>8))
			pixels = append(pixels, uint32(y), uint32(cb), uint32(cr))
		}
	}
	return Input{
		Source: source,
		YCbCr:  pixels,
		Width:  uint32(width),
		Height: uint32(height),
	}
}

// Process processes a frames
func (f *FrameProcessor) Process(output chan Frame) {
	nets, coords, net := &f.Nets, f.Coords, &f.Net
	query, key, value := []Matrix{}, []Matrix{}, []Matrix{}
	for in := range f.Input {
		width, height := int(in.Width), int(in.Height)
		if coords == nil {
			rng := rand.New(rand.NewSource(f.Seed))
			coords = make([][]Coord, len(*nets))
			for c := range coords {
				coords[c] = make([]Coord, Pixels)
				for x := 0; x < Pixels; x++ {
					coords[c][x].X = rng.Intn(width / 4)
					coords[c][x].Y = rng.Intn(height / 4)
				}
			}
			f.Coords = coords
		}
		for n := range *nets {
			input := NewMatrix(0, 3*Pixels, 1)
			for x := 0; x < Pixels; x++ {
				i := coords[n][x].X + (width/4)*(n%4)
				j := coords[n][x].Y + (height/4)*(n/4)
				y := in.YCbCr[3*j*width+3*i]
				cb := in.YCbCr[3*j*width+3*i+1]
				cr := in.YCbCr[3*j*width+3*i+2]
				fy, fcb, fcr := float64(y)/255, float64(cb)/255, float64(cr)/255
				input.Data = append(input.Data, float32(fy))
				input.Data = append(input.Data, float32(fcb))
				input.Data = append(input.Data, float32(fcr))
			}
			input = Normalize(input)
			q, k, v := (*nets)[n].Fire(input, input, input)
			query = append(query, q)
			key = append(key, k)
			value = append(value, v)
		}

		qq := NewMatrix(0, Nets*8, 1)
		for _, a := range query {
			for _, b := range a.Data {
				qq.Data = append(qq.Data, b)
			}
		}
		qq = Normalize(qq)

		kk := NewMatrix(0, Nets*8, 1)
		for _, a := range key {
			for _, b := range a.Data {
				kk.Data = append(kk.Data, b)
			}
		}
		kk = Normalize(kk)

		vv := NewMatrix(0, Nets*8, 1)
		for _, a := range value {
			for _, b := range a.Data {
				vv.Data = append(vv.Data, b)
			}
		}
		vv = Normalize(vv)

		q, k, v := net.Fire(qq, kk, vv)
		output <- Frame{
			Query: q,
			Key:   k,
			Value: v,
		}
	}
}

func picture() {
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
}

func main() {
	flag.Parse()

	if *FlagPicture {
		picture()
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
	pwm := 75
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
			in3.SetValue(1)
			in4.SetValue(0)
		case JoystickStateDown:
			in3.SetValue(0)
			in4.SetValue(1)
		default:
			in3.SetValue(0)
			in4.SetValue(0)
		}
		switch joystickLeft {
		case JoystickStateUp:
			in1.SetValue(1)
			in2.SetValue(0)
		case JoystickStateDown:
			in1.SetValue(0)
			in2.SetValue(1)
		default:
			in1.SetValue(0)
			in2.SetValue(0)
		}
	}

	go func() {
		rng := rand.New(rand.NewSource(32))
		actionsQ := make([][]float32, 8)
		for a := range actionsQ {
			vector := make([]float32, 32)
			for i := range vector {
				vector[i] = rng.Float32()
			}
			actionsQ[a] = vector
		}
		actionsK := make([][]float32, 8)
		for a := range actionsK {
			vector := make([]float32, 32)
			for i := range vector {
				vector[i] = rng.Float32()
			}
			actionsK[a] = vector
		}
		actionsV := make([][]float32, 8)
		for a := range actionsV {
			vector := make([]float32, 32)
			for i := range vector {
				vector[i] = rng.Float32()
			}
			actionsV[a] = vector
		}
		near := func(actions [][]float32, a []float32) int {
			max, index := float32(0.0), 0
			for j := range actions {
				b := actions[j]
				ab, aa, bb := float32(0.0), float32(0.0), float32(0.0)
				for k := range a {
					ab += a[k] * b[k]
					aa += a[k] * a[k]
					bb += b[k] * b[k]
				}
				s := ab / (float32(math.Sqrt(float64(aa))) * float32(math.Sqrt(float64(bb))))
				if s > max {
					max, index = s, j
				}
			}
			return index
		}
		center, centerProcessor, centerActivations := NewStreamCamera(), NewFrameProcessor(1), make(chan Frame, 8)
		left, leftProcessor, leftActivations := NewV4LCamera(), NewFrameProcessor(2), make(chan Frame, 8)
		right, rightProcessor, rightActivations := NewV4LCamera(), NewFrameProcessor(3), make(chan Frame, 8)

		go center.Start()
		go left.Start("/dev/videol")
		go right.Start("/dev/videor")

		query := NewMatrix(0, 3*Outputs+1, 1)
		query.Data = query.Data[:cap(query.Data)]
		key := NewMatrix(0, 3*Outputs+1, 1)
		key.Data = key.Data[:cap(key.Data)]
		value := NewMatrix(0, 3*Outputs+1, 1)
		value.Data = value.Data[:cap(value.Data)]
		out := NewNet(4, Window, 3*Outputs+1, 3*Outputs+32)
		setWindow := func(window int64) {
			out.SetWindow(window)
			for net := range centerProcessor.Nets {
				centerProcessor.Nets[net].SetWindow(window)
			}
			centerProcessor.Net.SetWindow(window)
			for net := range leftProcessor.Nets {
				leftProcessor.Nets[net].SetWindow(window)
			}
			leftProcessor.Net.SetWindow(window)
			for net := range rightProcessor.Nets {
				rightProcessor.Nets[net].SetWindow(window)
			}
			rightProcessor.Net.SetWindow(window)
		}
		go centerProcessor.Process(centerActivations)
		go leftProcessor.Process(leftActivations)
		go rightProcessor.Process(rightActivations)
		go func() {
			for running {
				select {
				case frame := <-center.Images:
					fmt.Println("center", frame.Frame.Bounds())
					centerProcessor.Input <- Convert(1, frame.Frame)
				case frame := <-left.Images:
					fmt.Println("left", frame.Frame.Bounds())
					leftProcessor.Input <- Convert(2, frame.Frame)
				case frame := <-right.Images:
					fmt.Println("right", frame.Frame.Bounds())
					rightProcessor.Input <- Convert(3, frame.Frame)
				}
			}
		}()
		for running {
			select {
			case frame := <-centerActivations:
				copy(query.Data[:Outputs], frame.Query.Data)
				copy(key.Data[:Outputs], frame.Key.Data)
				copy(value.Data[:Outputs], frame.Value.Data)
			case frame := <-leftActivations:
				copy(query.Data[Outputs:2*Outputs], frame.Query.Data)
				copy(key.Data[Outputs:2*Outputs], frame.Key.Data)
				copy(value.Data[Outputs:2*Outputs], frame.Value.Data)
			case frame := <-rightActivations:
				copy(query.Data[2*Outputs:3*Outputs], frame.Query.Data)
				copy(key.Data[2*Outputs:3*Outputs], frame.Key.Data)
				copy(value.Data[2*Outputs:3*Outputs], frame.Value.Data)
			}
			var votes [8]int
			query.Data[3*Outputs] = 0
			key.Data[3*Outputs] = 0
			value.Data[3*Outputs] = 0
			q, k, v := out.Fire(query, key, value)
			votes[near(actionsQ, q.Data[3*Outputs:])]++
			votes[near(actionsK, k.Data[3*Outputs:])]++
			votes[near(actionsV, v.Data[3*Outputs:])]++
			/*query.Data[3*Outputs] = 1
			copy(query.Data, q.Data[3*Outputs:])
			key.Data[3*Outputs] = 1
			copy(key.Data, k.Data[3*Outputs:])
			value.Data[3*Outputs] = 1
			copy(value.Data, v.Data[3*Outputs:])
			q, k, v = out.Fire(query, key, value)
			votes[near(actionsQ, q.Data[3*Outputs:])]++
			votes[near(actionsK, k.Data[3*Outputs:])]++
			votes[near(actionsV, v.Data[3*Outputs:])]++*/
			sum, index := 0, 0
			choice := rng.Intn(3)
			for k, v := range votes {
				sum += v
				if sum >= choice {
					index = k
					break
				}
			}
			fmt.Println("...............................................................................")
			fmt.Println(v.Data[3*Outputs:])
			if mode == ModeAuto {
				switch index {
				case 0:
					joystickLeft = JoystickStateUp
					joystickRight = JoystickStateUp
				case 1:
					joystickLeft = JoystickStateDown
					joystickRight = JoystickStateUp
				case 2:
					joystickLeft = JoystickStateUp
					joystickRight = JoystickStateDown
				case 3:
					joystickLeft = JoystickStateNone
					joystickRight = JoystickStateNone
				case 4:
					joystickLeft = JoystickStateDown
					joystickRight = JoystickStateDown
				case 5:
					setWindow(128)
				case 6:
					setWindow(32)
				case 7:
					setWindow(16)
				}
				update()
			}
		}
	}()

	pwmUpDownServo := 1500
	pwmLeftRightServo := 1500
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
