// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"
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
		nets[n] = NewNet(seed+1+int64(n), Window, 3*Pixels, 8)
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

			query, key, value := []Matrix{}, []Matrix{}, []Matrix{}
			img := videoFrame.Image()
			b := img.Bounds()
			width, height := b.Max.X, b.Max.Y
			if coords == nil {
				rng := rand.New(rand.NewSource(sc.Seed + int64(len(*nets))))
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
				q, k, v := (*nets)[n].Fire(input, input, input)
				query = append(query, q)
				key = append(key, k)
				value = append(value, v)
			}

			sum := 0.0
			qq := NewMatrix(0, Nets*8, 1)
			for _, a := range query {
				for _, b := range a.Data {
					sum += float64(b) * float64(b)
					qq.Data = append(qq.Data, b)
				}
			}
			length := math.Sqrt(sum)
			for i := range qq.Data {
				qq.Data[i] /= float32(length)
			}

			kk := NewMatrix(0, Nets*8, 1)
			for _, a := range key {
				for _, b := range a.Data {
					sum += float64(b) * float64(b)
					kk.Data = append(kk.Data, b)
				}
			}
			length = math.Sqrt(sum)
			for i := range kk.Data {
				kk.Data[i] /= float32(length)
			}

			vv := NewMatrix(0, Nets*8, 1)
			for _, a := range value {
				for _, b := range a.Data {
					sum += float64(b) * float64(b)
					vv.Data = append(vv.Data, b)
				}
			}
			length = math.Sqrt(sum)
			for i := range vv.Data {
				vv.Data[i] /= float32(length)
			}

			q, k, v := net.Fire(qq, kk, vv)
			select {
			case sc.Images <- Frame{
				Frame: videoFrame.Image(),
				//DCT:    pixels,
				Query: q,
				Key:   k,
				Value: v,
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
