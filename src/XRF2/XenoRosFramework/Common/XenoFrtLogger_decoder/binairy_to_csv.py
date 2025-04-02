import numpy as np
import csv

def binary_to_csv(binary_file_path, csv_file_path):
    with open(binary_file_path, 'rb') as f:
        # Read field names string ending with newline character
        field_names_str = f.readline().strip().decode('utf-8')
        field_names = field_names_str.split(',')
        
        # Read data types string ending with newline character
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
    
    # Read the rest of the binary data in chunks of row_size
    with open(binary_file_path, 'rb') as f:
        # Skip the field names and field types parts
        f.seek(len(field_names_str) + len(field_types_str) + 2)  # +2 for newline characters
        
        # Initialize data array
        data_array = []
        
        while True:
            chunk = f.read(row_size)
            if len(chunk) != row_size:
                break
            # Append chunk to data array
            data_array.append(np.frombuffer(chunk, dtype=dtypes))
    
    # Concatenate chunks into a single numpy array
    data_array = np.concatenate(data_array)
    
    # Write to CSV file
    with open(csv_file_path, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write header
        csvwriter.writerow(field_names)
        # Write data rows
        for row in data_array:
            csvwriter.writerow(row)

# Example usage
bin_file = input('Path to binary file (e.g., "log_data.bin"): ')
csv_file = input('Path to CSV file (e.g., "log_data.csv"): ')
binary_to_csv(bin_file, csv_file)
