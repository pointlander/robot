// Copyright 2023 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

//go:build 386 || arm || arm64
// +build 386 arm arm64

package matrix

func dot(X, Y []float32) float32 {
	var sum float32
	for i, x := range X {
		sum += x * Y[i]
	}
	return sum
}
