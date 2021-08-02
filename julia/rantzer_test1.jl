using LinearAlgebra
using JuMP
using Hypatia
using MinimaxAdaptiveControl

A = [3. -1;1 0]
B1 = Matrix{Float64}(undef,2,1)
B1[:,:] = [1. 0]
B2 = Matrix{Float64}(undef, 2,1)
B2[:,:] = [5 0]

As = [A,A]
Bs = [B1,B2]
Q = I(2)*1.
gamma = 22
R = I(1)*1.

mac = MAController(As, Bs, Q, R, gamma, [0.; 0])

