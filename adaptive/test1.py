import control
import numpy as np
from matplotlib import pyplot as plt

print("test1.py\n")

# Define Constants
K0 = 1
a0 = 1

# Define bounds on delta K
deltaK_lb = -0.5
deltaK_ub = 2.0

deltaa_lb = -2.0
deltaa_ub = 2.0

deltaK = np.random.uniform(deltaK_lb,deltaK_ub)
deltaa = np.random.uniform(deltaa_lb,deltaa_ub)

# Create values for K and a
K = K0 + deltaK
a = a0 + deltaa

G0 = control.TransferFunction([K],[1,a])

print(G0)

print("Poles + zero info:")
print(control.tf2ss(G0).pole())
print(control.zero(G0))

dt = 0.1
T0 = np.arange(0.0,1.0,dt)
T, yout = control.step_response(G0,T=T0)

print(len(yout))

fig = plt.figure()
plt.plot(T0,yout)

# Create figures for multiple values of a

# Create n step responses
n_responses = 3
fig2 = plt.figure()
Ka_labels = []
for a_index in range(0,n_responses):
    deltaK = np.random.uniform(deltaK_lb,deltaK_ub)
    deltaa = np.random.uniform(deltaa_lb,deltaa_ub)    

    # Create values for K and a
    K = K0 + deltaK
    a = a0 + deltaa

    temp_label = str(K)+'-'+str(a)

    Gi = control.TransferFunction([K],[1,a])

    T, yout = control.step_response(Gi,T=T0)
    plt.plot(T0,yout,label=temp_label)

    Ka_labels = [ Ka_labels , temp_label ]

plt.legend()
plt.show()


