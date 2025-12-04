import os

# The exact structure we agreed on
structure = {
    "firmware_stm32/Core": [],
    "firmware_stm32/Drivers": [],
    "firmware_stm32/Middlewares": [],
    "firmware_stm32/src": ["main.c", "pwm_control.c", "uart_protocol.c"],
    
    "src_python": ["main.py", "config.py"],
    "src_python/perception": ["detector.py", "traffic_light.py", "lane_finder.py", "sensor_fusion.py"],
    "src_python/control": ["uart_handler.py", "pid_controller.py", "curve_logic.py"],
    "src_python/utils": ["logger.py", "camera_stream.py"],
    
    "models": ["best_yolo.pt", "traffic_light_cnn.pt", "road_lane.onnx"],
    
    "tools_and_tests": ["collect_data.py", "crop_dataset.py", "test_uart_echo.py"],
    "tools_and_tests/unit_tests": [],
    
    "docs/hardware": [],
    "docs/software": [],
}

def create_structure():
    print("🚀 Generating BFMC-Optinx Repository Structure...")
    
    for folder, files in structure.items():
        # Create the directory
        os.makedirs(folder, exist_ok=True)
        print(f"   📂 Created: {folder}")
        
        # Create a .gitkeep file so Git tracks empty folders
        with open(os.path.join(folder, ".gitkeep"), "w") as f:
            pass
            
        # Create specific files
        for file in files:
            file_path = os.path.join(folder, file)
            if not os.path.exists(file_path):
                with open(file_path, "w") as f:
                    # Add a placeholder comment so the file isn't totally empty
                    f.write(f"# Placeholder for {file}\n")
                print(f"      📄 Created: {file}")

    # Create root files
    root_files = [".gitignore", "README.md", "requirements.txt"]
    for rf in root_files:
        if not os.path.exists(rf):
            with open(rf, "w") as f:
                if rf == ".gitignore":
                    f.write("__pycache__/\n*.pyc\n*.o\n*.bin\n/firmware_stm32/Debug/\n*.pt\n")
                elif rf == "README.md":
                    f.write("# BFMC 2025 - Team Optinx\n\nOfficial Repository.")
            print(f"   📄 Created Root File: {rf}")

    print("\n✅ Done! Structure is ready.")

if __name__ == "__main__":
    create_structure()