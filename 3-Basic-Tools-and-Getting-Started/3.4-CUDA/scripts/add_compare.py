from numba import cuda, float32
import numpy as np
import time


@cuda.jit
def add_gpu(a, b, out):
    tx = cuda.threadIdx.x
    ty = cuda.blockIdx.x
    block_size = cuda.blockDim.x
    grid_size = cuda.gridDim.x
    start = tx + ty * block_size
    stride = block_size * grid_size

    for i in range(start, a.shape[0], stride):
        out[i] = a[i] + b[i]

def add_cpu(a, b, out):
    for i in range(a.shape[0]):    
        out[i] = a[i] + b[i]

def main():
    # Prepare test data
    n = 10000000
    a = np.arange(n).astype(np.float32)
    b = np.arange(n).astype(np.float32)
    out = np.empty_like(a)

    # 1. Test CPU computation time
    start = time.time()
    add_cpu(a, b, out)
    print("CPU Time Taken:", time.time()-start)

    # 2. Test GPU computation time
    # Copy data from host memory to device memory
    d_a = cuda.to_device(a)
    d_b = cuda.to_device(b)
    d_out = cuda.device_array_like(out)
    # Define the number of threads per block and the number of blocks
    threads_per_block = 256  
    blocks_per_grid = (n + threads_per_block - 1) // threads_per_block
    # Compute the result
    start_gpu = time.time()
    add_gpu[blocks_per_grid, threads_per_block](d_a, d_b, d_out)
    end_gpu = time.time()
    # Copy the result from device memory back to host memory
    d_out.copy_to_host(out)

    print("GPU Time Taken:", end_gpu-start_gpu)


if __name__ == "__main__":
    main()
