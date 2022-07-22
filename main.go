// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"

	"github.com/veandco/go-sdl2/sdl"
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
					fmt.Printf("speed %d\n", speed)
				}

				switch joystickRight {
				case JoystickStateUp:
					fmt.Println("right up")
				case JoystickStateDown:
					fmt.Println("right down")
				}
				switch joystickLeft {
				case JoystickStateUp:
					fmt.Println("left up")
				case JoystickStateDown:
					fmt.Println("left down")
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
