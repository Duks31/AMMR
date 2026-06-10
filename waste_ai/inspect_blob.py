import depthai as dai

# Path to the blob file you want to investigate
BLOB_PATH = "models/taco_2class_300epoch.blob" 

try:
    # Load the Myriad X blob file
    blob = dai.OpenVINO.Blob(BLOB_PATH)
    
    print("\n=== 📥 NETWORK INPUTS ===")
    for name, tensor in blob.networkInputs.items():
        print(f"🔹 Input Layer Name : {name}")
        print(f"   Shape            : {tensor.dims} (Layout Order: {tensor.order})")
        print(f"   Data Type        : {tensor.dataType}")
    
    print("\n=== 📤 NETWORK OUTPUTS ===")
    for name, tensor in blob.networkOutputs.items():
        print(f"🔸 Output Layer Name: {name}")
        print(f"   Shape            : {tensor.dims}")
        print(f"   Data Type        : {tensor.dataType}")
        print("-" * 30)

except Exception as e:
    print(f"Error loading or reading the blob file: {e}")
