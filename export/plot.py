import numpy as np
import matplotlib.pyplot as plt

loaded_array = np.load('output_traj.npy')
# breakpoint()
converstion = 180/np.pi
i = 1
breakpoint()
for i in range(loaded_array.shape[2]):
    fig, axs = plt.subplots(2, 1)

    axs[0].plot(loaded_array[:,0,i], loaded_array[:,1,i]*converstion)
    axs[0].set_ylabel('thetas')
    axs[0].set_xlabel('time')
    axs[0].grid(True)

    axs[1].plot(loaded_array[:,0,i], loaded_array[:,2,i]*converstion)
    axs[1].set_ylabel('theta velocities')
    axs[1].set_xlabel('time')
    axs[1].grid(True)

    fig.suptitle('plots for joint: {}'.format(i))
    name = "output_figs/{}.png".format(i+1)
    plt.savefig(name)
    # plt.show()

# breakpoint()