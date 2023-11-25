// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"
	"image"
	"image/color"
	"io"
	"math"
	"math/cmplx"
	"os"
	"os/exec"
	"time"

	. "github.com/pointlander/robot/matrix"

	"github.com/mjibson/go-dsp/fft"
	"github.com/nfnt/resize"
	"github.com/zergon321/reisen"
)

// StreamCamera is a camera that is from a stream
type StreamCamera struct {
	Stream bool
	Images chan Frame
	Seed   int64
	Net    Net
}

// NewStreamCamera creates a new streaming camera
func NewStreamCamera(seed int64) *StreamCamera {
	return &StreamCamera{
		Stream: true,
		Images: make(chan Frame, 8),
		Seed:   seed,
		Net:    NewNet(seed, Window, Inputs, Outputs),
	}
}

// Start starts streaming
func (sc *StreamCamera) Start() {
	net := &sc.Net
	skip := 0
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

	start, count := time.Now(), 0.0
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
			count++

			fmt.Println("center", count/float64(time.Since(start).Seconds()))

			if skip < 10 {
				skip++
			} else {
				skip = 0
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
			sum := 0.0
			input := NewMatrix(0, Inputs, 1)
			for _, a := range pixels {
				for _, b := range a {
					sum += b
					input.Data = append(input.Data, float32(b))
				}
			}
			length := math.Sqrt(sum)
			for i := range input.Data {
				input.Data[i] /= float32(length)
			}
			select {
			case sc.Images <- Frame{
				Frame:  videoFrame.Image(),
				DCT:    pixels,
				Output: net.Fire(input),
			}:
			default:
				fmt.Println("drop center")
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
