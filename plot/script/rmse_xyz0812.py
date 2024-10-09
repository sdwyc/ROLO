import numpy as np
import pandas as pd

def load_tum_file(filepath):
    """
    Load a TUM format file into a pandas DataFrame.
    """
    return pd.read_csv(filepath, sep='\s+', header=None, names=['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

def sync_files(file1, file2):
    """
    Synchronize the rows of file2 to match the timestamps in file1 by finding the closest timestamps.
    """
    synced_file2 = []
    for _, row in file1.iterrows():
        time_diff = np.abs(file2['time'] - row['time'])
        closest_idx = time_diff.idxmin()
        synced_file2.append(file2.loc[closest_idx])
    return pd.DataFrame(synced_file2).reset_index(drop=True)

def calculate_rmse(file1_xyz, file2_xyz):
    """
    Calculate the Root Mean Square Error (RMSE) between two sets of XYZ coordinates.
    """
    error = file1_xyz - file2_xyz
    squared_error = np.square(error)
    mean_squared_error = np.mean(squared_error, axis=0)
    rmse = np.sqrt(mean_squared_error)
    return rmse

def main(file1_path, file2_path):
    # Load the TUM files
    file1 = load_tum_file(file1_path)
    file2 = load_tum_file(file2_path)

    # Synchronize the files by timestamps
    file2_synced = sync_files(file1, file2)

    # Extract XYZ coordinates
    file1_xyz = file1[['x', 'y', 'z']].to_numpy()
    file2_xyz = file2_synced[['x', 'y', 'z']].to_numpy()

    # Calculate RMSE
    rmse = calculate_rmse(file1_xyz, file2_xyz)

    # Output the RMSE for x, y, z
    print(f"RMSE for X: {rmse[0]:.6f}")
    print(f"RMSE for Y: {rmse[1]:.6f}")
    print(f"RMSE for Z: {rmse[2]:.6f}")

# Replace with your file paths
file1_path = r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\xinglong\FAST-LIO2.tum"
file2_path = r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\xinglong\GroundTruth.tum"

if __name__ == "__main__":
    main(file1_path, file2_path)
