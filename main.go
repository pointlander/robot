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
	"math"
	"os"
	"sort"
	"time"

	"github.com/pointlander/gradient/tf32"
	"github.com/pointlander/occam"

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
	Memory = 8
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

// Frame is a video frame
type Frame struct {
	Frame image.Image
	DCT   [][]float64
}

// Column is like a brain column
type Column struct {
	Net   *occam.Network
	Max   float32
	Index int
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

	go func() {
		stream := NewStreamCamera()
		left := NewV4LCamera()
		right := NewV4LCamera()
		go stream.Start()
		go left.Start("/dev/videol")
		go right.Start("/dev/videor")

		var time float64
		w := Width*Height + 4
		columns := [8]Column{}
		for i := range columns {
			columns[i].Net = occam.NewNetwork(w, 3*Memory)
			columns[i].Max = -1
		}
		var Indexes [3]int
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
			max, c := float32(0.0), 0
			for i := range columns {
				if columns[i].Max < 0 {
					c = i
					break
				} else if columns[i].Max > max {
					max = columns[i].Max
					c = i
				}
			}
			net := columns[c].Net
			offset, i := Memory*index*w+Indexes[index]*w, 0
			for y := 0; y < Height; y++ {
				for x := 0; x < Width; x++ {
					net.Point.X[offset+i] = float32(line[y][x])
					i++
				}
			}
			net.Point.X[offset+Width*Height] = 0
			net.Point.X[offset+Width*Height+1] = 0
			net.Point.X[offset+Width*Height+2] = 0
			net.Point.X[offset+Width*Height+index] = 1
			net.Point.X[offset+Width*Height+3] = float32(math.Sin(2 * time * math.Pi))
			Indexes[index] = (Indexes[index] + 1) % Memory

			max, index = float32(0.0), 0
			for i := 0; i < 3*Memory; i++ {
				for i, value := range net.Point.X[i*w : (i+1)*w] {
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
			if index < Memory {
				index = 0
			} else if index < 2*Memory {
				index = 1
			} else {
				index = 2
			}
			columns[c].Max = max
			columns[c].Index = index

			sort.Slice(columns[:], func(i, j int) bool {
				return columns[i].Max > columns[j].Max
			})
			var history [States]int
			for i := range columns[:3] {
				history[columns[i].Index]++
			}
			m, c := 0, 0
			for i := range history {
				if history[i] > m {
					m, c = history[i], i
					break
				}
			}
			index = columns[c].Index
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
			time += 1 / 256.0
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
