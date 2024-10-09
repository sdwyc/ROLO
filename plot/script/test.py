import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

def load_tum_file(filepath):
    """
    Load a TUM format file into a pandas DataFrame.
    """
    return pd.read_csv(filepath, sep='\s+', header=None, names=['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

def quaternion_to_euler(qx, qy, qz, qw):
    """
    Convert a quaternion (qx, qy, qz, qw) to euler angles (roll, pitch, yaw).
    The euler angles are returned in radians and constrained within the range [-pi, pi].
    """
    rotation = R.from_quat([qx, qy, qz, qw])
    euler_angles = rotation.as_euler('xyz', degrees=False)
    
    # Ensure the euler angles are within the range [-pi, pi]
    euler_angles = np.mod(euler_angles + np.pi, 2 * np.pi) - np.pi
    return euler_angles

def convert_tum_to_euler(file):
    """
    Convert the quaternion columns of a TUM file to euler angles and return a new DataFrame.
    """
    euler_data = []
    for _, row in file.iterrows():
        euler_angles = quaternion_to_euler(row['qx'], row['qy'], row['qz'], row['qw'])
        euler_data.append([row['time'], euler_angles[0], euler_angles[1], euler_angles[2]])
    
    euler_df = pd.DataFrame(euler_data, columns=['time', 'roll', 'pitch', 'yaw'])
    return euler_df

def save_to_tum_file(filepath, euler_df):
    """
    Save the euler angle DataFrame to a TUM format file.
    """
    euler_df.to_csv(filepath, sep=' ', index=False, header=False, float_format='%.6f')

def main(input_file_path, output_file_path):
    # Load the TUM file
    tum_file = load_tum_file(input_file_path)
    
    # Convert quaternion to euler angles
    euler_df = convert_tum_to_euler(tum_file)
    
    # Save the euler angles to a new TUM file
    save_to_tum_file(output_file_path, euler_df)

# 示例用法
input_file_path =  r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\qianfo\FAST-LIO2.tum"
output_file_path = r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\euler\qianfo\ol-FAST-LIO2.tum"

if __name__ == "__main__":
    main(input_file_path, output_file_path)
