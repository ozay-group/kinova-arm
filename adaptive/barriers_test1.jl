using DynamicalSystems, Plots


# Lorenz system
# Equations of motion:
@inline @inbounds function loop(u, p, t)
    σ = p[1]; ρ = p[2]; β = p[3]
    du1 = σ*(u[2]-u[1])
    du2 = u[1]*(ρ-u[3]) - u[2]
    du3 = u[1]*u[2] - β*u[3]
    return SVector{3}(du1, du2, du3)
end
# Jacobian:
@inline @inbounds function loop_jac(u, p, t)
    σ, ρ, β = p
    J = @SMatrix [-σ  σ  0;
    ρ - u[3]  (-1)  (-u[1]);
    u[2]   u[1]  -β]
    return J
end

ds = ContinuousDynamicalSystem(loop, rand(3), [10.0, 28.0, 8/3], loop_jac)

dataset0 = trajectory(ds, 1.0)
print(dataset0)

# plot3d(dataset0)
Plots.plot(dataset0[:,1],dataset0[:,2])
plot3d(dataset0[:,1],dataset0[:,2],dataset0[:,3])
