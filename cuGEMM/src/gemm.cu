#include <cuda_runtime.h>
#include <iostream>

__global__ void cuGEMM(const float *A, const float *B, float *C, int N, int K, int M) {
    // Calculate row and column index of the element in the output matrix C
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int col = blockIdx.x * blockDim.x + threadIdx.x;

    if (row < N && col < M) {
        float value = 0.0f;
        // Compute the dot product of the row of A and column of B
        for (int i = 0; i < K; ++i) {
            value += A[row * K + i] * B[i * M + col];
        }
        // Write the result to the output matrix C
        C[row * M + col] = value;
    }
}

void launchGEMM(const float *A, const float *B, float *C, int N, int K, int M) {
    float *d_A, *d_B, *d_C;
    cudaMalloc((void**)&d_A, N * K * sizeof(float));
    cudaMalloc((void**)&d_B, K * M * sizeof(float));
    cudaMalloc((void**)&d_C, N * M * sizeof(float));

    cudaMemcpy(d_A, A, N * K * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_B, B, K * M * sizeof(float), cudaMemcpyHostToDevice);

    // Define block and grid dimensions
    dim3 blockSize(16, 16); // 16x16 threads per block
    dim3 gridSize((M + blockSize.x - 1) / blockSize.x, (N + blockSize.y - 1) / blockSize.y);

    // Launch the kernel
    cuGEMM<<<gridSize, blockSize>>>(d_A, d_B, d_C, N, K, M);

    // Copy the result back to the host
    cudaMemcpy(C, d_C, N * M * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(d_A);
    cudaFree(d_B);
    cudaFree(d_C);
}
