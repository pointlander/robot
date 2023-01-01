// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"
	"image"
	"image/color"
	"math/cmplx"
	"runtime"
	"sort"

	"github.com/blackjack/webcam"
	"github.com/mjibson/go-dsp/fft"
	"github.com/nfnt/resize"
)

// FrameSizes is a slice of FrameSize
type FrameSizes []webcam.FrameSize

// Len is the length of the slice
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
				fmt.Println("drop", device)
			}
		}
	}
}
