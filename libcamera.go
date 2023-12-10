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
	"math/rand"
	"os"
	"os/exec"
	"time"

	. "github.com/pointlander/matrix"

	"github.com/zergon321/reisen"
)

// StreamCamera is a camera that is from a stream
type StreamCamera struct {
	Stream bool
	Images chan Frame
	Seed   int64
	Net    Net
	Nets   []Net
}

// NewStreamCamera creates a new streaming camera
func NewStreamCamera(seed int64) *StreamCamera {
	nets := make([]Net, Nets)
	for n := range nets {
		nets[n] = NewNet(seed+1+int64(n), Window, 256, 8)
	}
	return &StreamCamera{
		Stream: true,
		Images: make(chan Frame, 8),
		Seed:   seed,
		Net:    NewNet(seed, Window, Nets*8, Outputs),
		Nets:   nets,
	}
}

// Start starts streaming
func (sc *StreamCamera) Start() {
	net := &sc.Net
	nets := &sc.Nets
	var coords [][]Coord

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

			outputs := []Matrix{}
			img := videoFrame.Image()
			b := img.Bounds()
			gray := image.NewGray(b)
			for y := 0; y < b.Max.Y; y++ {
				for x := 0; x < b.Max.X; x++ {
					original := img.At(x, y)
					pixel := color.GrayModel.Convert(original)
					gray.Set(x, y, pixel)
				}
			}
			width, height := b.Max.X, b.Max.Y
			if coords == nil {
				rng := rand.New(rand.NewSource(sc.Seed + int64(len(*nets))))
				coords = make([][]Coord, len(*nets))
				for c := range coords {
					coords[c] = make([]Coord, 256)
					for x := 0; x < 256; x++ {
						coords[c][x].X = rng.Intn(width / 4)
						coords[c][x].Y = rng.Intn(height / 4)
					}
				}
			}
			for n := range *nets {
				input, sum := NewMatrix(0, 256, 1), 0.0
				for x := 0; x < 256; x++ {
					pixel := gray.GrayAt(coords[n][x].X /*(*nets)[n].Rng.Intn(width/4)*/ +(width/4)*(n%4),
						coords[n][x].Y /*(*nets)[n].Rng.Intn(height/4)*/ +(height/4)*(n/4))
					input.Data = append(input.Data, float32(pixel.Y))
					sum += float64(pixel.Y) * float64(pixel.Y)
				}
				length := math.Sqrt(sum)
				for x := range input.Data {
					input.Data[x] /= float32(length)
				}
				outputs = append(outputs, (*nets)[n].Fire(input))
			}

			/*tiny := resize.Resize(Width, Height, videoFrame.Image(), resize.Lanczos3)
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
			}*/
			/*output := fft.FFT2Real(pixels)
			for j, pix := range pixels {
				for i := range pix {
					pix[i] = cmplx.Abs(output[j][i]) / float64(width*height)
				}
			}*/
			sum := 0.0
			input := NewMatrix(0, Nets*8, 1)
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
			case sc.Images <- Frame{
				Frame: videoFrame.Image(),
				//DCT:    pixels,
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
