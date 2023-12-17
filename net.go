// Copyright 2022 The Robot Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"math"
	"math/rand"
	"sort"
	"sync/atomic"

	. "github.com/pointlander/matrix"
)

const (
	// Samples is the number of samples
	Samples = 256
)

// Random is a random variable
type Random struct {
	Mean   float32
	StdDev float32
}

// Set is a set of statistics
type Set [][]Random

// NewStatistics generates a new statistics model
func NewStatistics(inputs, outputs int) Set {
	statistics := make(Set, outputs)
	for i := range statistics {
		for j := 0; j < inputs; j++ {
			statistics[i] = append(statistics[i], Random{
				Mean:   0,
				StdDev: 1,
			})
		}
	}
	return statistics
}

// Sample samples from the statistics
func (s Set) Sample(rng *rand.Rand, inputs, outputs int) []Matrix {
	neurons := make([]Matrix, outputs)
	for j := range neurons {
		neurons[j] = NewMatrix(0, inputs, 1)
		for k := 0; k < inputs; k++ {
			v := float32(rng.NormFloat64())*s[j][k].StdDev + s[j][k].Mean
			if v > 0 {
				v = 1
			} else {
				v = -1
			}
			neurons[j].Data = append(neurons[j].Data, v)
		}
	}
	return neurons
}

// Net is a net
type Net struct {
	window  int64
	Inputs  int
	Outputs int
	Rng     *rand.Rand
	Q       Set
	K       Set
	V       Set
}

// NewNet makes a new network
func NewNet(seed int64, window int64, inputs, outputs int) Net {
	rng := rand.New(rand.NewSource(seed))
	return Net{
		window:  window,
		Inputs:  inputs,
		Outputs: outputs,
		Rng:     rng,
		Q:       NewStatistics(inputs, outputs),
		K:       NewStatistics(inputs, outputs),
		V:       NewStatistics(inputs, outputs),
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

// CalculateStatistics calculates the statistics of systems
func (n Net) CalculateStatistics(systems []Sample) Set {
	window := atomic.LoadInt64(&n.window)
	statistics := make(Set, n.Outputs)
	for i := range statistics {
		for j := 0; j < n.Inputs; j++ {
			statistics[i] = append(statistics[i], Random{
				Mean:   0,
				StdDev: 0,
			})
		}
	}
	for i := range systems[:window] {
		for j := range systems[i].Neurons {
			for k, value := range systems[i].Neurons[j].Data {
				statistics[j][k].Mean += value
			}
		}
	}
	for i := range statistics {
		for j := range statistics[i] {
			statistics[i][j].Mean /= float32(window)
		}
	}
	for i := range systems[:window] {
		for j := range systems[i].Neurons {
			for k, value := range systems[i].Neurons[j].Data {
				diff := statistics[j][k].Mean - value
				statistics[j][k].StdDev += diff * diff
			}
		}
	}
	for i := range statistics {
		for j := range statistics[i] {
			statistics[i][j].StdDev /= float32(window)
			statistics[i][j].StdDev = float32(math.Sqrt(float64(statistics[i][j].StdDev)))
		}
	}
	return statistics
}

// Fire runs the network
func (n *Net) Fire(input Matrix) Matrix {
	q := NewMatrix(0, n.Outputs+n.Inputs, Samples)
	k := NewMatrix(0, n.Outputs+n.Inputs, Samples)
	v := NewMatrix(0, n.Outputs+n.Inputs, Samples)
	systemsQ := make([]Sample, 0, 8)
	systemsK := make([]Sample, 0, 8)
	systemsV := make([]Sample, 0, 8)
	for i := 0; i < Samples; i++ {
		neurons := n.Q.Sample(n.Rng, n.Inputs, n.Outputs)
		outputs := NewMatrix(0, n.Outputs, 1)
		for j := range neurons {
			out := MulT(neurons[j], input)
			q.Data = append(q.Data, out.Data[0])
			outputs.Data = append(outputs.Data, out.Data[0])
		}
		a, b := make([]float64, n.Inputs), make([]float64, n.Inputs)
		for j := range neurons {
			for jj, value := range neurons[j].Data {
				if value < 0 {
					a[jj]++
				} else {
					b[jj]++
				}
			}
		}
		for j, a := range a {
			b := b[j]
			a, b = a/(a+b), b/(a+b)
			q.Data = append(q.Data, -float32(a*math.Log(a)+b*math.Log(b)))
		}
		systemsQ = append(systemsQ, Sample{
			Neurons: neurons,
			Outputs: outputs,
		})
	}
	for i := 0; i < Samples; i++ {
		neurons := n.K.Sample(n.Rng, n.Inputs, n.Outputs)
		outputs := NewMatrix(0, n.Outputs, 1)
		for j := range neurons {
			out := MulT(neurons[j], input)
			k.Data = append(k.Data, out.Data[0])
			outputs.Data = append(outputs.Data, out.Data[0])
		}
		a, b := make([]float64, n.Inputs), make([]float64, n.Inputs)
		for j := range neurons {
			for jj, value := range neurons[j].Data {
				if value < 0 {
					a[jj]++
				} else {
					b[jj]++
				}
			}
		}
		for j, a := range a {
			b := b[j]
			a, b = a/(a+b), b/(a+b)
			k.Data = append(k.Data, -float32(a*math.Log(a)+b*math.Log(b)))
		}
		systemsK = append(systemsK, Sample{
			Neurons: neurons,
			Outputs: outputs,
		})
	}
	for i := 0; i < Samples; i++ {
		neurons := n.V.Sample(n.Rng, n.Inputs, n.Outputs)
		outputs := NewMatrix(0, n.Outputs, 1)
		for j := range neurons {
			out := MulT(neurons[j], input)
			v.Data = append(v.Data, out.Data[0])
			outputs.Data = append(outputs.Data, out.Data[0])
		}
		a, b := make([]float64, n.Inputs), make([]float64, n.Inputs)
		for j := range neurons {
			for jj, value := range neurons[j].Data {
				if value < 0 {
					a[jj]++
				} else {
					b[jj]++
				}
			}
		}
		for j, a := range a {
			b := b[j]
			a, b = a/(a+b), b/(a+b)
			v.Data = append(v.Data, -float32(a*math.Log(a)+b*math.Log(b)))
		}
		systemsV = append(systemsV, Sample{
			Neurons: neurons,
			Outputs: outputs,
		})
	}
	entropies := SelfEntropy(q, k, v)
	for i, entropy := range entropies {
		systemsQ[i].Entropy = entropy
		systemsK[i].Entropy = entropy
		systemsV[i].Entropy = entropy
	}
	sort.Slice(systemsQ, func(i, j int) bool {
		return systemsQ[i].Entropy < systemsQ[j].Entropy
	})
	sort.Slice(systemsK, func(i, j int) bool {
		return systemsK[i].Entropy < systemsK[j].Entropy
	})
	sort.Slice(systemsV, func(i, j int) bool {
		return systemsV[i].Entropy < systemsV[j].Entropy
	})

	nq := n.CalculateStatistics(systemsQ)
	for i, v := range nq {
		for j, vv := range v {
			n.Q[i][j].Mean = (1-Rate)*n.Q[i][j].Mean + Rate*vv.Mean
			n.Q[i][j].StdDev = (1-Rate)*n.Q[i][j].StdDev + Rate*vv.StdDev
		}
	}
	nk := n.CalculateStatistics(systemsK)
	for i, v := range nk {
		for j, vv := range v {
			n.K[i][j].Mean = (1-Rate)*n.K[i][j].Mean + Rate*vv.Mean
			n.K[i][j].StdDev = (1-Rate)*n.K[i][j].StdDev + Rate*vv.StdDev
		}
	}
	nv := n.CalculateStatistics(systemsV)
	for i, v := range nv {
		for j, vv := range v {
			n.V[i][j].Mean = (1-Rate)*n.V[i][j].Mean + Rate*vv.Mean
			n.V[i][j].StdDev = (1-Rate)*n.V[i][j].StdDev + Rate*vv.StdDev
		}
	}
	return systemsV[0].Outputs
}
