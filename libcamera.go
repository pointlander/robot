// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"io"
	"os"
	"os/exec"
	"time"

	"github.com/zergon321/reisen"
)

// StreamCamera is a camera that is from a stream
type StreamCamera struct {
	Stream bool
	Images chan Frame
}

// NewStreamCamera creates a new streaming camera
func NewStreamCamera() *StreamCamera {
	return &StreamCamera{
		Stream: true,
		Images: make(chan Frame, 1),
	}
}

// Start starts streaming
func (sc *StreamCamera) Start() {
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
	_ = start
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

			//fmt.Println("center", count/float64(time.Since(start).Seconds()))

			if skip < 10 {
				skip++
			} else {
				skip = 0
				continue
			}

			select {
			case sc.Images <- Frame{
				Frame: videoFrame.Image(),
			}:
			default:
				//fmt.Println("drop center")
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
