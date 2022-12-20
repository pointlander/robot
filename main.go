// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"
	"image"
	"image/color"
	"image/png"
	"math/cmplx"
	"os"
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

func main() {
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

	go func() {
		runtime.LockOSThread()
		camera, err := webcam.Open("/dev/video0")
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

		net := occam.NewNetwork(64, 9)

		var cp []byte
		first := false
		for running {
			err = camera.WaitForFrame(5)

			switch err.(type) {
			case nil:
			case *webcam.Timeout:
				fmt.Fprint(os.Stderr, err.Error())
				continue
			default:
				panic(err.Error())
			}

			frame, err := camera.ReadFrame()
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
				tiny := resize.Resize(24, 24, yuyv, resize.Lanczos3)
				b := tiny.Bounds()
				gray := image.NewGray(b)
				for y := 0; y < b.Max.Y; y++ {
					for x := 0; x < b.Max.X; x++ {
						original := tiny.At(x, y)
						pixel := color.GrayModel.Convert(original)
						gray.Set(x, y, pixel)
					}
				}
				if !first {
					first = true
					output, err := os.Create("test.png")
					if err != nil {
						panic(err)
					}
					defer output.Close()
					png.Encode(output, gray)
				}
				width, height, index := b.Max.X, b.Max.Y, 0
				pixels := make([][]float64, height)
				for i := range pixels {
					pixels[i] = make([]float64, width)
				}
				for i := 0; i < width; i += 8 {
					for j := 0; j < height; j += 8 {
						for x := 0; x < 8; x++ {
							for y := 0; y < 8; y++ {
								pixels[y][x] = float64(gray.At(i+x, j+y).(color.Gray).Y) / 255
							}
						}
						output := fft.FFT2Real(pixels)
						for x := 0; x < 8; x++ {
							for y := 0; y < 8; y++ {
								net.Point.X[index] = float32(cmplx.Abs(output[y][x]) / float64(width*height))
								index++
							}
						}
					}
				}
				max, index := float32(0.0), 0
				for i := 0; i < 9; i++ {
					for i, value := range net.Point.X[i*32 : (i+1)*32] {
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
				if mode == ModeAuto {
					switch index {
					case 0:
						fmt.Println("Left")
						joystickLeft = JoystickStateUp
						joystickRight = JoystickStateDown
					case 1:
						fmt.Println("Forward")
						joystickLeft = JoystickStateUp
						joystickRight = JoystickStateUp
					case 2:
						fmt.Println("Right")
						joystickLeft = JoystickStateDown
						joystickRight = JoystickStateUp
					case 3:
						fmt.Println("Left Forward")
						joystickLeft = JoystickStateUp
						joystickRight = JoystickStateUp
					case 4:
						fmt.Println("Forward")
						joystickLeft = JoystickStateUp
						joystickRight = JoystickStateUp
					case 5:
						fmt.Println("Right Forward")
						joystickLeft = JoystickStateUp
						joystickRight = JoystickStateUp
					case 6:
						fmt.Println("Left Backward")
						joystickLeft = JoystickStateDown
						joystickRight = JoystickStateNone
					case 7:
						fmt.Println("Backward")
						joystickLeft = JoystickStateDown
						joystickRight = JoystickStateDown
					case 8:
						fmt.Println("Right Backward")
						joystickLeft = JoystickStateNone
						joystickRight = JoystickStateDown
					}
					update()
				}
			} else if err != nil {
				panic(err)
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
