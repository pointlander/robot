// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"
	"image"
	"image/color"
	"math"
	"math/rand"
	"runtime"
	"sort"
	"time"

	. "github.com/pointlander/matrix"

	"github.com/blackjack/webcam"
)

// FrameSizes is a slice of FrameSize
type FrameSizes []webcam.FrameSize

// Len is the length of the slice
func (slice FrameSizes) Len() int {
	return len(slice)
}

// For sorting purposes
func (slice FrameSizes) Less(i, j int) bool {
	ls := slice[i].MaxWidth * slice[i].MaxHeight
	rs := slice[j].MaxWidth * slice[j].MaxHeight
	return ls < rs
}

// For sorting purposes
func (slice FrameSizes) Swap(i, j int) {
	slice[i], slice[j] = slice[j], slice[i]
}

// V4LCamera is a camera that is from a v4l device
type V4LCamera struct {
	Stream bool
	Images chan Frame
	Seed   int64
	Net    Net
	Nets   []Net
}

// NewV4LCamera creates a new v4l camera
func NewV4LCamera(seed int64) *V4LCamera {
	nets := make([]Net, Nets)
	for n := range nets {
		nets[n] = NewNet(seed+1+int64(n), Window, 3*Pixels, 8)
	}
	return &V4LCamera{
		Stream: true,
		Images: make(chan Frame, 8),
		Seed:   seed,
		Net:    NewNet(seed, Window, 3*Nets*8, Outputs),
		Nets:   nets,
	}
}

// Start starts streaming
func (vc *V4LCamera) Start(device string) {
	net := &vc.Net
	nets := &vc.Nets
	var coords [][]Coord

	runtime.LockOSThread()
	skip := 0
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
	start, count := time.Now(), 0.0
	for vc.Stream {
		err := camera.WaitForFrame(5)

		switch err.(type) {
		case nil:
		case *webcam.Timeout:
			fmt.Println(device, err)
			continue
		default:
			panic(err)
		}

		frame, err := camera.ReadFrame()
		if err != nil {
			fmt.Println(device, err)
			continue
		} else {
			fmt.Println(device)
		}
		count++

		fmt.Println(device, count/float64(time.Since(start).Seconds()))

		if skip < 20 {
			skip++
		} else {
			skip = 0
			continue
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

			outputs := []Matrix{}
			img := yuyv
			b := img.Bounds()
			width, height := b.Max.X, b.Max.Y
			if coords == nil {
				rng := rand.New(rand.NewSource(vc.Seed + int64(len(*nets))))
				coords = make([][]Coord, len(*nets))
				for c := range coords {
					coords[c] = make([]Coord, Pixels)
					for x := 0; x < Pixels; x++ {
						coords[c][x].X = rng.Intn(width / 4)
						coords[c][x].Y = rng.Intn(height / 4)
					}
				}
			}
			for n := range *nets {
				input, sum := NewMatrix(0, 3*Pixels, 1), 0.0
				for x := 0; x < Pixels; x++ {
					pixel := img.At(coords[n][x].X+(width/4)*(n%4), coords[n][x].Y+(height/4)*(n/4))
					r, g, b, _ := pixel.RGBA()
					y, cb, cr := color.RGBToYCbCr(uint8(r>>8), uint8(g>>8), uint8(b>>8))
					fy, fcb, fcr := float64(y)/255, float64(cb)/255, float64(cr)/255
					input.Data = append(input.Data, float32(fy))
					input.Data = append(input.Data, float32(fcb))
					input.Data = append(input.Data, float32(fcr))
					sum += fy*fy + fcb*fcb + fcr*fcr
				}
				length := math.Sqrt(sum)
				for x := range input.Data {
					input.Data[x] /= float32(length)
				}
				outputs = append(outputs, (*nets)[n].Fire(input))
			}

			sum := 0.0
			input := NewMatrix(0, 3*Nets*8, 1)
			for _, a := range outputs {
				for _, b := range a.Data {
					sum += float64(b) * float64(b)
					input.Data = append(input.Data, b)
				}
			}
			length := math.Sqrt(sum)
			for i := range input.Data {
				input.Data[i] /= float32(length)
			}
			select {
			case vc.Images <- Frame{
				Frame: yuyv,
				//DCT:    pixels,
				Output: net.Fire(input),
			}:
			default:
				fmt.Println("drop", device)
			}
		}
	}
}
