// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"
	"time"

	"github.com/veandco/go-sdl2/sdl"
	"github.com/warthog618/gpiod"
	"github.com/warthog618/gpiod/device/rpi"
)

var joysticks = make(map[int]*sdl.Joystick)

// JoystickState is the state of a joystick
type JoystickState uint

const (
	// JoystickStateNone is the default state of a joystick
	JoystickStateNone JoystickState = iota
	// JoystickStateUp is the state of a joystick when it is pushed up
	JoystickStateUp
	// JoystickStateDown is the state of a joystick when it is pushed down
	JoystickStateDown
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

	for running {
		for event = sdl.PollEvent(); event != nil; event = sdl.PollEvent() {
			switch t := event.(type) {
			case *sdl.QuitEvent:
				running = false
			case *sdl.JoyAxisEvent:
				value := int16(t.Value)
				axis[t.Axis] = value
				if t.Axis == 3 || t.Axis == 4 {
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
					//fmt.Printf("right [%d ms] Which: %v \t%d %d\n",
					//		t.Timestamp, t.Which, axis[3], axis[4])
				} else if t.Axis == 0 || t.Axis == 1 {
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
					//fmt.Printf("left [%d ms] Which: %v \t%d %d\n",
					//t.Timestamp, t.Which, axis[0], axis[1])
				} else if t.Axis == 2 {
					//fmt.Printf("2 axis [%d ms] Which: %v \t%x\n",
					//	t.Timestamp, t.Which, value)
					speed = axis[2]
					pwm = int(100 * (float64(speed) + 32768) / 65535)
					fmt.Printf("speed %d pwm %d\n", speed, pwm)
				}

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
			case *sdl.JoyBallEvent:
				fmt.Printf("[%d ms] Ball:%d\txrel:%d\tyrel:%d\n",
					t.Timestamp, t.Ball, t.XRel, t.YRel)
			case *sdl.JoyButtonEvent:
				fmt.Printf("[%d ms] Button:%d\tstate:%d\n",
					t.Timestamp, t.Button, t.State)
			case *sdl.JoyHatEvent:
				fmt.Printf("[%d ms] Hat:%d\tvalue:%d\n",
					t.Timestamp, t.Hat, t.Value)
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
