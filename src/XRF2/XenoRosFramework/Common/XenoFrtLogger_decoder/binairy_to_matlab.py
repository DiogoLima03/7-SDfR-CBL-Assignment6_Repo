import numpy as np
import scipy.io

def binary_to_mat(binary_file_path, mat_file_path):
    # Open the binary file to read field names and types
    with open(binary_file_path, 'rb') as f:
        field_names_str = f.readline().strip().decode('utf-8')
        field_names = field_names_str.split(',')
        
        field_types_str = f.readline().strip().decode('utf-8')
        field_types = field_types_str.split(',')

    # Debugging: print the extracted field names and types
    print("Field Names:", field_names)
    print("Field Types:", field_types)

    # Ensure the number of field names matches the number of field types
    if len(field_names) != len(field_types):
        raise ValueError("The number of field names does not match the number of field types")

    # Map field types to numpy dtypes based on numerical values
    type_map = {
        '1': np.bool8,
        '2': np.dtype('S1'),  # single character
        '3': np.int32,
        '4': np.float32,
        '5': np.float64,
        '6': np.int64
    }

    try:
        dtypes = [(field_names[i], type_map[field_types[i]]) for i in range(len(field_names))]
    except KeyError as e:
        raise ValueError(f"Invalid field type encountered: {e}")

    # Calculate the size of each row
    row_size = sum(np.dtype(dtype).itemsize for _, dtype in dtypes)
    print("Expected row size:", row_size)
    
    # Initialize dictionary to store the results
    data_dict = {name: [] for name in field_names}
    
    # Read the binary data in chunks and append to corresponding lists
    with open(binary_file_path, 'rb') as f:
        # Skip the field names and field types parts
        f.seek(len(field_names_str) + len(field_types_str) + 2)  # +2 for newline characters
        
        while True:
            chunk = f.read(row_size)
            if len(chunk) != row_size:
                break
            # Convert chunk to a structured numpy array with one row
            row_array = np.frombuffer(chunk, dtype=dtypes, count=1)
            for name in field_names:
                data_dict[name].append(row_array[name][0])
    
    # Convert lists in the dictionary to numpy arrays
    for name in field_names:
        data_dict[name] = np.array(data_dict[name])
    
    # Write to MAT file
    scipy.io.savemat(mat_file_path, data_dict)

# Example usage
bin_file = input('Path to binary file (e.g., "log_data.bin"): ')
mat_file = input('Path to MAT file (e.g., "log_data.mat"): ')
binary_to_mat(bin_file, mat_file)
