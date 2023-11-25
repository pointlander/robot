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
	"math/rand"
	"os"
	"sort"
	"sync/atomic"
	"time"

	"github.com/pointlander/occam"
	. "github.com/pointlander/robot/matrix"

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
	// Width is the width of the fft
	Width = 24
	// Height is the height of the fft
	Height = 24
	// States is the number of states
	States = 3
	// Memory is the sive of memory per state
	Memory = 8
	// NetWidth is the width of the network
	NetWidth = Width*Height + 5
)

const (
	// Window is the window size
	Window = 32
	// Samples is the number of samples
	Samples = 256
	// Inputs is the number of inputs
	Inputs = Width * Height
	// Outputs is the number of outputs
	Outputs = 32
)

// Random is a random variable
type Random struct {
	Mean   float32
	StdDev float32
}

// Net is a net
type Net struct {
	window       int64
	Inputs       int
	Outputs      int
	Rng          *rand.Rand
	Distribution [][]Random
}

// NewNet makes a new network
func NewNet(seed int64, window int64, inputs, outputs int) Net {
	rng := rand.New(rand.NewSource(seed))
	distribution := make([][]Random, outputs)
	for i := range distribution {
		for j := 0; j < inputs; j++ {
			distribution[i] = append(distribution[i], Random{
				Mean:   0,
				StdDev: 1,
			})
		}
	}
	return Net{
		window:       window,
		Inputs:       inputs,
		Outputs:      outputs,
		Rng:          rng,
		Distribution: distribution,
	}
}

// Set window sets the window
func (n *Net) SetWindow(window int64) {
	atomic.StoreInt64(&n.window, window)
}

// Sample is a sample of a random neural network
type Sample struct {
	Entropy float32
	Neurons []Matrix
	Outputs Matrix
}

// Fire runs the network
func (n *Net) Fire(input Matrix) Matrix {
	rng, distribution, window := n.Rng, n.Distribution, atomic.LoadInt64(&n.window)
	output := NewMatrix(0, n.Outputs, Samples)

	systems := make([]Sample, 0, 8)
	for i := 0; i < Samples; i++ {
		neurons := make([]Matrix, n.Outputs)
		for j := range neurons {
			neurons[j] = NewMatrix(0, n.Inputs, 1)
			for k := 0; k < n.Inputs; k++ {
				v := float32(rng.NormFloat64())*distribution[j][k].StdDev + distribution[j][k].Mean
				if v > 0 {
					v = 1
				} else {
					v = -1
				}
				neurons[j].Data = append(neurons[j].Data, v)
			}
		}
		outputs := NewMatrix(0, n.Outputs, 1)
		for j := range neurons {
			out := MulT(neurons[j], input)
			output.Data = append(output.Data, out.Data[0])
			outputs.Data = append(outputs.Data, out.Data[0])
		}
		systems = append(systems, Sample{
			Neurons: neurons,
			Outputs: outputs,
		})
	}
	entropies := SelfEntropy(output, output, output)
	for i, entropy := range entropies {
		systems[i].Entropy = entropy
	}
	sort.Slice(entropies, func(i, j int) bool {
		return systems[i].Entropy < systems[j].Entropy
	})
	next := make([][]Random, n.Outputs)
	for i := range next {
		for j := 0; j < n.Inputs; j++ {
			next[i] = append(next[i], Random{
				Mean:   0,
				StdDev: 0,
			})
		}
	}
	for i := range systems[:window] {
		for j := range systems[i].Neurons {
			for k, value := range systems[i].Neurons[j].Data {
				next[j][k].Mean += value
			}
		}
	}
	for i := range next {
		for j := range next[i] {
			next[i][j].Mean /= float32(window)
		}
	}
	for i := range systems[:window] {
		for j := range systems[i].Neurons {
			for k, value := range systems[i].Neurons[j].Data {
				diff := next[j][k].Mean - value
				next[j][k].StdDev += diff * diff
			}
		}
	}
	for i := range next {
		for j := range next[i] {
			next[i][j].StdDev /= float32(window)
			next[i][j].StdDev = float32(math.Sqrt(float64(next[i][j].StdDev)))
		}
	}
	n.Distribution = next
	return systems[0].Outputs
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

// Entropy is the self entropy of a point
type Entropy struct {
	Entropy float32
	Camera  TypeCamera
}

// Column is like a brain column
type Column struct {
	Net     *occam.Network
	Max     float32
	Camera  TypeCamera
	Indexes [States]int
	Entropy [3 * Memory]Entropy
	Split   int
}

func split(entropy []Entropy) int {
	sum := float32(0.0)
	for _, e := range entropy {
		sum += e.Entropy
	}
	avg, vari := sum/float32(len(entropy)), float32(0.0)
	for _, e := range entropy {
		difference := e.Entropy - avg
		vari += difference * difference
	}
	vari /= float32(len(entropy))

	index, max := 0, float32(0.0)
	for i := 1; i < len(entropy); i++ {
		suma, counta := float32(0.0), float32(0.0)
		for _, e := range entropy[:i] {
			suma += e.Entropy
			counta++
		}
		avga, varia := suma/counta, float32(0.0)
		for _, e := range entropy[:i] {
			difference := e.Entropy - avga
			varia += difference * difference
		}
		varia /= counta

		sumb, countb := float32(0.0), float32(0.0)
		for _, e := range entropy[i:] {
			sumb += e.Entropy
			countb++
		}
		avgb, varib := sumb/countb, float32(0.0)
		for _, e := range entropy[i:] {
			difference := e.Entropy - avgb
			varib += difference * difference
		}
		varib /= countb

		gain := vari - (varia + varib)
		if gain > max {
			index, max = i, gain
		}
	}
	return index
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
		//rnd := rand.New(rand.NewSource(1))

		center := NewStreamCamera(1)
		left := NewV4LCamera(2)
		right := NewV4LCamera(3)
		go center.Start()
		go left.Start("/dev/videol")
		go right.Start("/dev/videor")

		//var time float64
		columns := [8]Column{}
		for i := range columns {
			columns[i].Net = occam.NewNetwork(NetWidth, 3*Memory)
			columns[i].Max = 8
		}
		//current := TypeCameraNone
		output := NewMatrix(0, 3*Outputs, 1)
		output.Data = output.Data[:cap(output.Data)]
		out := NewNet(4, Window, 3*Outputs, 3)
		win := NewNet(5, Window, 3*Outputs+3, 3)
		metaInput := NewMatrix(0, 3*Outputs+3, 1)
		metaInput.Data = metaInput.Data[:cap(metaInput.Data)]
		for running {
			select {
			case frame := <-center.Images:
				copy(output.Data[:Outputs], frame.Output.Data)
			case frame := <-left.Images:
				copy(output.Data[Outputs:2*Outputs], frame.Output.Data)
			case frame := <-right.Images:
				copy(output.Data[2*Outputs:3*Outputs], frame.Output.Data)
			}
			a := out.Fire(output)
			fmt.Println("...............................................................................")
			fmt.Println(a.Data)
			{
				copy(metaInput.Data[:3*Outputs], output.Data)
				copy(metaInput.Data[3*Outputs:], a.Data)
				w := win.Fire(metaInput)
				max, index := float32(0.0), 0
				for i, v := range w.Data {
					if v > max {
						max, index = v, i
					}
				}
				switch index {
				case 0:
					center.Net.SetWindow(8)
					left.Net.SetWindow(8)
					right.Net.SetWindow(8)
					out.SetWindow(8)
				case 1:
					center.Net.SetWindow(16)
					left.Net.SetWindow(16)
					right.Net.SetWindow(16)
					out.SetWindow(16)
				case 2:
					center.Net.SetWindow(32)
					left.Net.SetWindow(32)
					right.Net.SetWindow(32)
					out.SetWindow(32)
				}
			}
			max, index := 0.0, 0
			for i, v := range a.Data {
				if float64(v) > max {
					max, index = float64(v), i
				}
			}
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
				}
				update()
			}

			/*var line [][]float64
			var camera TypeCamera
			select {
			case frame := <-center.Images:
				if current == TypeCameraCenter {
					current = TypeCameraNone
				} else {
					camera = TypeCameraCenter
				}
				line = frame.DCT
			case frame := <-left.Images:
				if current == TypeCameraLeft {
					current = TypeCameraNone
				} else {
					camera = TypeCameraLeft
				}
				line = frame.DCT
			case frame := <-right.Images:
				if current == TypeCameraRight {
					current = TypeCameraNone
				} else {
					camera = TypeCameraRight
				}
				line = frame.DCT
			}
			sort.Slice(columns[:], func(i, j int) bool {
				return columns[i].Max > columns[j].Max
			})
			total := float32(0.0)
			for i := range columns {
				total += columns[i].Max
			}
			c, b := 0, rnd.Float32()
			bin := float32(0.0)
			for i := range columns {
				bin += columns[i].Max / total
				if b < bin {
					c = i
					break
				}
			}
			net := columns[c].Net
			offset, i := Memory*int(camera)*NetWidth+columns[c].Indexes[camera]*NetWidth, 0
			for y := 0; y < Height; y++ {
				for x := 0; x < Width; x++ {
					net.Point.X[offset+i] = float32(line[y][x])
					i++
				}
			}
			net.Point.X[offset+Width*Height] = 0
			net.Point.X[offset+Width*Height+1] = 0
			net.Point.X[offset+Width*Height+2] = 0
			net.Point.X[offset+Width*Height+3] = 0
			net.Point.X[offset+Width*Height+int(camera)] = 1
			net.Point.X[offset+Width*Height+4] = float32(math.Sin(2 * time * math.Pi))
			columns[c].Indexes[camera] = (columns[c].Indexes[camera] + 1) % Memory

			max, index := float32(0.0), 0
			for i := 0; i < 3*Memory; i++ {
				for i, value := range net.Point.X[i*NetWidth : (i+1)*NetWidth] {
					net.Input.X[i] = float32(value)
				}
				net.Cost(func(a *tf32.V) bool {
					columns[c].Entropy[i].Entropy = float32(a.X[0])
					columns[c].Entropy[i].Camera = TypeCamera(i / Memory)
					if a.X[0] > max {
						max = a.X[0]
						index = i
					}
					return true
				})
			}
			sort.Slice(columns[c].Entropy[:], func(i, j int) bool {
				return columns[c].Entropy[i].Entropy > columns[c].Entropy[j].Entropy
			})
			columns[c].Split = split(columns[c].Entropy[:])

			if index < Memory {
				camera = TypeCameraCenter
			} else if index < 2*Memory {
				camera = TypeCameraLeft
			} else {
				camera = TypeCameraRight
			}
			columns[c].Max = max
			columns[c].Camera = camera
			sort.Slice(columns[:], func(i, j int) bool {
				return columns[i].Max > columns[j].Max
			})

			var history [4]float32*/
			/*for i := range columns {
				if columns[i].Max < 0 {
					continue
				}
				history[columns[i].Camera] += columns[i].Max
			}*/
			/*for i := range columns {
				for j := range columns[i].Entropy[:columns[i].Split] {
					history[columns[i].Entropy[j].Camera] += columns[i].Entropy[j].Entropy
				}
			}
			max, camera = float32(0.0), TypeCameraNone
			for i, value := range history {
				if value > max {
					max = value
					camera = TypeCamera(i)
				}
			}
			current = camera
			if mode == ModeAuto {
				switch camera {
				case TypeCameraCenter:
					fmt.Println("Forward")
					joystickLeft = JoystickStateUp
					joystickRight = JoystickStateUp
				case TypeCameraLeft:
					fmt.Println("Left")
					joystickLeft = JoystickStateDown
					joystickRight = JoystickStateUp
				case TypeCameraRight:
					fmt.Println("Right")
					joystickLeft = JoystickStateUp
					joystickRight = JoystickStateDown
				case TypeCameraNone:
				}
				update()
			}
			time += 1 / 256.0*/
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
