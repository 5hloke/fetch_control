import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    #load up the .npy files

    des_pos = np.load('des_position.npy')
    des_vel = np.load('des_velocity.npy')

    actual_pos = np.load('actual_position.npy')
    actual_vel = np.load('actual_velocities.npy')

    error_pos = np.load('error_position.npy')
    error_vel = np.load('error_velocity.npy')

    for i in range(7):

        plt.figure()
        plt.plot(des_pos[:, i], label='desired position')
        plt.plot(actual_pos[:, i], label='actual position')
        plt.legend()
        plt.title(f'Joint {i} position')
        plt.xlabel('time')
        plt.ylabel('position')

        plt.figure()
        plt.plot(des_vel[:, i], label='desired velocity')
        plt.plot(actual_vel[:, i], label='actual velocity')
        plt.legend()
        plt.title(f'Joint {i} velocity')
        plt.xlabel('time')
        plt.ylabel('velocity')
        
        plt.figure()
        plt.plot(error_pos[:, i], label='position error')
        plt.plot(error_vel[:, i], label='velocity error')
        plt.legend()
        plt.title(f'Joint {i} error')
        plt.xlabel('time')
        plt.ylabel('error')
        # compute the RMSE velocity and position
        rmse_pos = np.sqrt(np.mean(np.square(error_pos[:, i])))
        rmse_vel = np.sqrt(np.mean(np.square(error_vel[:, i])))
        print(f'Joint {i} position RMSE: {rmse_pos}')
        print(f'Joint {i} velocity RMSE: {rmse_vel}')



    # plt.show()
