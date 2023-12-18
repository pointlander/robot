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
	Pixels = 32
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
	Frame  image.Image
	DCT    [][]float64
	Output Matrix
}

func picture() {
	stream := NewStreamCamera(1)
	left := NewV4LCamera(2)
	right := NewV4LCamera(3)
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
		center := NewStreamCamera(1)
		left := NewV4LCamera(2)
		right := NewV4LCamera(3)
		go center.Start()
		go left.Start("/dev/videol")
		go right.Start("/dev/videor")

		output := NewMatrix(0, 3*3*Outputs, 1)
		output.Data = output.Data[:cap(output.Data)]
		out := NewNet(4, Window, 3*3*Outputs, 3)
		setWindow := func(window int64) {
			out.SetWindow(window)
			for net := range center.Nets {
				center.Nets[net].SetWindow(window)
			}
			center.Net.SetWindow(window)
			for net := range left.Nets {
				left.Nets[net].SetWindow(window)
			}
			left.Net.SetWindow(window)
			for net := range right.Nets {
				right.Nets[net].SetWindow(window)
			}
			right.Net.SetWindow(window)
		}
		for running {
			select {
			case frame := <-center.Images:
				fmt.Println("center", frame.Frame.Bounds())
				// 640,480
				copy(output.Data[:3*Outputs], frame.Output.Data)
			case frame := <-left.Images:
				fmt.Println("left", frame.Frame.Bounds())
				// 320,240
				copy(output.Data[3*Outputs:3*2*Outputs], frame.Output.Data)
			case frame := <-right.Images:
				fmt.Println("right", frame.Frame.Bounds())
				// 320,240
				copy(output.Data[3*2*Outputs:3*3*Outputs], frame.Output.Data)
			}
			a := out.Fire(output)
			a = Normalize(a)
			fmt.Println("...............................................................................")
			fmt.Println(a.Data)
			if mode == ModeAuto {
				/*sum := [3]float32{}
				for i := range sum {
					sum[i] += a.Data[i] + a.Data[i+3] + a.Data[i+6]
				}*/
				c := 0
				for i, v := range a.Data[6:] {
					if v > 0 {
						c |= 1 << i
					}
				}
				switch c {
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
					setWindow(64)
				case 7:
					setWindow(32)
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
