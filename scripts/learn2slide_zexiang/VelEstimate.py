# import numpy as np
# from scipy.signal import savgol_filter
# import matplotlib.pyplot as plt

# def velocity_estimator(times, positions, window_size=11, poly_order=3, plot_results=False):
#     """
#     Estimate the velocity of an object from noisy position data using Savitzky-Golay filter.
    
#     Parameters:
#     times (array-like): Array of time data.
#     positions (array-like): Array of position data corresponding to the time data.
#     window_size (int): Window size for the Savitzky-Golay filter. Default is 11.
#     poly_order (int): Polynomial order for the Savitzky-Golay filter. Default is 3.
#     plot_results (bool): Whether to plot the results. Default is False.
    
#     Returns:
#     smoothed_positions (np.array): Smoothed position data.
#     smoothed_velocity (np.array): Smoothed velocity data.
#     """
    
#     # Step 1: Smooth the position data
#     smoothed_positions = savgol_filter(positions, window_size, poly_order)
    
#     # Step 2: Differentiate the smoothed data to find velocity
#     time_diffs = np.diff(times)
#     position_diffs = np.diff(smoothed_positions)
#     velocity = position_diffs / time_diffs
    
#     # Step 3: Further smooth the velocity data if necessary
#     smoothed_velocity = savgol_filter(velocity, window_size, poly_order)
    
#     # Plotting the results if requested
#     if plot_results:
#         plt.figure(figsize=(12, 6))
        
#         # Original and smoothed positions
#         plt.subplot(2, 1, 1)
#         plt.plot(times, positions, label='Noisy Positions')
#         plt.plot(times, smoothed_positions, label='Smoothed Positions', linewidth=2)
#         plt.xlabel('Time')
#         plt.ylabel('Position')
#         plt.legend()
        
#         # Velocity
#         plt.subplot(2, 1, 2)
#         plt.plot(times[:0], velocity, label='Velocity')
#         plt.plot(times[:-1], smoothed_velocity, label='Smoothed Velocity', linewidth=2)
#         plt.xlabel('Time')
#         plt.ylabel('Velocity')
#         plt.legend()
        
#         plt.tight_layout()
#         plt.show()
    
#     return smoothed_positions, smoothed_velocity

# if __name__ == "__main__":
#     times = np.linspace(0, 10, 100)
#     positions = np.sin(times) + np.random.normal(0, 0.1, 100)  # Noisy sine wave
#     smoothed_positions, smoothed_velocity = velocity_estimator(times.tolist(), positions.tolist(), plot_results=True)

import numpy as np
import matplotlib.pyplot as plt

def moving_average(data, window_size):
    """
    Apply a simple moving average to the data.
    
    Parameters:
    data (array-like): The input data to be smoothed.
    window_size (int): The size of the moving average window.
    
    Returns:
    np.array: The smoothed data.
    """
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

def velocity_estimator(times, positions, window_size=3, plot_results=False):
    """
    Estimate the velocity of an object from noisy position data using moving average smoothing.
    
    Parameters:
    times (array-like): Array of time data.
    positions (array-like): Array of position data corresponding to the time data.
    window_size (int): Window size for the moving average filter. Default is 5.
    plot_results (bool): Whether to plot the results. Default is False.
    
    Returns:
    smoothed_positions (np.array): Smoothed position data.
    smoothed_velocity (np.array): Smoothed velocity data.
    """
    
    # Step 1: Smooth the position data using a moving average
    smoothed_positions = moving_average(positions, window_size)
    
    # Adjust times array to match the length of the smoothed positions
    adjusted_times = times[:len(smoothed_positions)]
    
    # Step 2: Differentiate the smoothed data to find velocity
    time_diffs = np.diff(adjusted_times)
    position_diffs = np.diff(smoothed_positions)
    velocity = position_diffs / time_diffs
    
    # Step 3: Smooth the velocity data using a moving average
    smoothed_velocity = moving_average(velocity, window_size)
    
    # Adjust times array to match the length of the smoothed velocity
    final_times = adjusted_times[:len(smoothed_velocity)]
    
    # Plotting the results if requested
    if plot_results:
        plt.figure(figsize=(12, 6))
        
        # Original and smoothed positions
        plt.subplot(2, 1, 1)
        plt.plot(times, positions, label='Noisy Positions')
        plt.plot(adjusted_times, smoothed_positions, label='Smoothed Positions', linewidth=2)
        plt.xlabel('Time')
        plt.ylabel('Position')
        plt.legend()
        
        # Velocity
        plt.subplot(2, 1, 2)
        plt.plot(final_times, smoothed_velocity, label='Smoothed Velocity', linewidth=2)
        plt.xlabel('Time')
        plt.ylabel('Velocity')
        plt.legend()
        
        plt.tight_layout()
        plt.show()
    
    return smoothed_positions, smoothed_velocity