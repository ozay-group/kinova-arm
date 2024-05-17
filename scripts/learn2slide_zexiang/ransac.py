import numpy as np
def distance(vec1, vec2):
    """Calculate the Euclidean distance between two vectors."""
    return np.linalg.norm(vec1 - vec2)

def ransac(data, sample_ratio = 0.2, num_iterations=100, threshold=10):
    """
    Use RANSAC to average a set of vectors while ruling out outliers.

    Parameters:
    - data: List of numpy arrays representing vectors.
    - num_iterations: Number of iterations to run RANSAC.
    - threshold: Threshold to determine inliers.

    Returns:
    - final_model: The averaged vector considered as the final model.
    - final_inliers: List of vectors considered as inliers.
    """

    X = np.vstack(data)
    min_inliers = len(data) // 2  # Minimum number of inliers required
    sample_size = int(sample_ratio*len(data))

    best_model = None
    best_inliers = []

    for _ in range(num_iterations):
        # Randomly sample a subset of data points
        indices = np.random.choice(len(data), size=min(3, sample_size), replace=False)
        sample = X[indices]

        # Determine inliers
        distances = np.array([distance(x, sample.mean(axis=0)) for x in X])
        inliers = np.where(distances < threshold)[0]

        # Refit the model using inliers
        if len(inliers) >= min_inliers:
            model = X[inliers].mean(axis=0)
            if len(inliers) > len(best_inliers):
                best_model = model
                best_inliers = inliers

    final_model = best_model
    final_inliers = [data[i] for i in best_inliers]

    return final_model, final_inliers

if __name__ == "__main__":
    # Example usage:
    data = [np.array([1, 2, 3]), np.array([2, 3, 4]), np.array([3, 4, 5]),
            np.array([20, 30, 40]), np.array([100, 200, 300]), np.array([5, 6, 7])]

    final_model, final_inliers = ransac(data)
    print("Final model:", final_model)
    print("Inliers:", final_inliers)