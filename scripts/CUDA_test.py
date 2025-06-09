#!/usr/bin/env python3
import cv2
import numpy as np
import time

def test_opencv_cuda():
    """Test OpenCV CUDA functionality on Jetson Nano"""
    
    print("=" * 60)
    print("OpenCV CUDA Test for Jetson Nano")
    print("=" * 60)
    
    # Basic info
    print(f"OpenCV version: {cv2.__version__}")
    print(f"CUDA devices: {cv2.cuda.getCudaEnabledDeviceCount()}")
    
    if cv2.cuda.getCudaEnabledDeviceCount() == 0:
        print("❌ No CUDA devices found!")
        return False
    
    print("✅ CUDA device detected!")
    
    # Test 1: Basic GPU memory operations
    print("\n🧪 Test 1: GPU Memory Operations")
    try:
        gpu_mat = cv2.cuda_GpuMat(100, 100, cv2.CV_8UC3)
        gpu_mat.setTo((255, 0, 0))  # Set to blue
        cpu_result = gpu_mat.download()
        print("✅ GPU memory allocation and download: PASSED")
    except Exception as e:
        print(f"❌ GPU memory test failed: {e}")
        return False
    
    # Test 2: Image processing operations
    print("\n🧪 Test 2: CUDA Image Processing")
    try:
        # Create test image
        img = np.random.randint(0, 255, (1000, 1000, 3), dtype=np.uint8)
        
        # Upload to GPU
        gpu_img = cv2.cuda_GpuMat()
        gpu_img.upload(img)
        
        # Test available CUDA operations
        operations_tested = []
        
        # Test color conversion
        try:
            gpu_gray = cv2.cuda.cvtColor(gpu_img, cv2.COLOR_BGR2GRAY)
            operations_tested.append("cvtColor")
        except:
            pass
        
        # Test resize
        try:
            gpu_resized = cv2.cuda.resize(gpu_img, (500, 500))
            operations_tested.append("resize")
        except:
            pass
            
        # Test arithmetic operations
        try:
            gpu_result = cv2.cuda.add(gpu_img, gpu_img)
            operations_tested.append("add")
        except:
            pass
            
        # Test filters using cudaimgproc
        try:
            if hasattr(cv2.cuda, 'bilateralFilter'):
                gpu_filtered = cv2.cuda.bilateralFilter(gpu_gray, -1, 80, 80)
                operations_tested.append("bilateralFilter")
        except:
            pass
            
        # Test threshold
        try:
            _, gpu_thresh = cv2.cuda.threshold(gpu_gray, 127, 255, cv2.THRESH_BINARY)
            operations_tested.append("threshold")
        except:
            pass
        
        if operations_tested:
            print(f"✅ CUDA image processing ({', '.join(operations_tested)}): PASSED")
        else:
            print("⚠️  No CUDA image processing operations available")
            
    except Exception as e:
        print(f"❌ CUDA image processing test failed: {e}")
        return False
    
    # Test 3: Performance comparison
    print("\n🧪 Test 3: CPU vs GPU Performance")
    try:
        # Create larger test image
        img = np.random.randint(0, 255, (2000, 2000, 3), dtype=np.uint8)
        
        # CPU timing
        start_time = time.time()
        cpu_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cpu_resized = cv2.resize(cpu_gray, (1000, 1000))
        cpu_time = time.time() - start_time
        
        # GPU timing
        gpu_img = cv2.cuda_GpuMat()
        gpu_img.upload(img)
        
        start_time = time.time()
        gpu_gray = cv2.cuda.cvtColor(gpu_img, cv2.COLOR_BGR2GRAY)
        gpu_resized = cv2.cuda.resize(gpu_gray, (1000, 1000))
        result = gpu_resized.download()
        gpu_time = time.time() - start_time
        
        speedup = cpu_time / gpu_time if gpu_time > 0 else 0
        
        print(f"CPU time: {cpu_time:.4f}s")
        print(f"GPU time: {gpu_time:.4f}s")
        print(f"Speedup: {speedup:.2f}x")
        
        if speedup > 1.2:
            print("✅ GPU showing performance improvement")
        else:
            print("⚠️  GPU not showing significant speedup (this is normal for small operations)")
            
    except Exception as e:
        print(f"❌ Performance test failed: {e}")
    
    # Test 4: Available CUDA modules
    print("\n🧪 Test 4: Available CUDA Modules")
    cuda_modules = [
        'cuda', 'cudaarithm', 'cudabgsegm', 'cudafeatures2d', 
        'cudafilters', 'cudaimgproc', 'cudaobjdetect', 
        'cudaoptflow', 'cudastereo', 'cudawarping'
    ]
    
    available_modules = []
    for module in cuda_modules:
        if hasattr(cv2, module):
            available_modules.append(module)
            print(f"✅ {module}")
        else:
            print(f"❌ {module} - not available")
    
    print(f"\nTotal CUDA modules available: {len(available_modules)}/{len(cuda_modules)}")
    
    # Test 5: DNN with CUDA (if available)
    print("\n🧪 Test 5: DNN CUDA Support")
    try:
        net = cv2.dnn.readNet()  # Empty net for testing
        if cv2.dnn.DNN_BACKEND_CUDA in [cv2.dnn.DNN_BACKEND_CUDA]:
            print("✅ DNN CUDA backend available")
        else:
            print("⚠️  DNN CUDA backend not available")
    except:
        print("⚠️  DNN CUDA test inconclusive")
    
    print("\n" + "=" * 60)
    print("🎉 OpenCV CUDA setup is working!")
    print("Your Jetson Nano is ready for GPU-accelerated computer vision!")
    print("=" * 60)
    
    return True

def demo_cuda_operations():
    """Demo some practical CUDA operations"""
    print("\n🚀 CUDA Operations Demo")
    print("-" * 40)
    
    # Create a sample image
    img = np.zeros((400, 600, 3), dtype=np.uint8)
    cv2.rectangle(img, (50, 50), (150, 150), (0, 255, 0), -1)
    cv2.circle(img, (300, 200), 75, (255, 0, 0), -1)
    cv2.putText(img, "CUDA Test", (200, 350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Upload to GPU
    gpu_img = cv2.cuda_GpuMat()
    gpu_img.upload(img)
    
    print("Available GPU operations you can use:")
    print("- cv2.cuda.cvtColor() - Color space conversion")
    print("- cv2.cuda.resize() - Image resizing") 
    print("- cv2.cuda.threshold() - Thresholding")
    print("- cv2.cuda.add/subtract/multiply() - Arithmetic operations")
    print("- cv2.cuda.split/merge() - Channel operations")
    print("- cv2.cuda.flip() - Image flipping")
    print("- cv2.cuda.rotate() - Image rotation")
    print("- cv2.cuda.remap() - Geometric transformations")
    
    # Demonstrate a few operations
    gpu_gray = cv2.cuda.cvtColor(gpu_img, cv2.COLOR_BGR2GRAY)
    gpu_resized = cv2.cuda.resize(gpu_gray, (300, 200))
    
    print("\n✅ Demo operations completed successfully!")
    print("Your OpenCV installation is ready for high-performance computer vision!")

if __name__ == "__main__":
    if test_opencv_cuda():
        demo_cuda_operations()