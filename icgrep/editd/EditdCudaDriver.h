#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cassert>
#include "cuda.h"

#define GROUPTHREADS 64
#define GROUPBLOCKS 64

void checkCudaErrors(CUresult err) {
  assert(err == CUDA_SUCCESS);
}

/// main - Program entry point
ulong * RunPTX(std::string PTXFilename, char * fileBuffer, ulong filesize, const char * patternStr, unsigned patternLen, int dist) {
  
  CUdevice    device;
  CUmodule    cudaModule;
  CUcontext   context;
  CUfunction  function;
  int         devCount;

  // CUDA initialization
  checkCudaErrors(cuInit(0));
  checkCudaErrors(cuDeviceGetCount(&devCount));
  checkCudaErrors(cuDeviceGet(&device, 0));

  char name[128];
  checkCudaErrors(cuDeviceGetName(name, 128, device));
  // std::cout << "Using CUDA Device [0]: " << name << "\n";

  int devMajor, devMinor;
  checkCudaErrors(cuDeviceComputeCapability(&devMajor, &devMinor, device));
  // std::cout << "Device Compute Capability: " << devMajor << "." << devMinor << "\n";
  if (devMajor < 2) {
    std::cerr << "ERROR: Device 0 is not SM 2.0 or greater\n";
    exit(-1);
  }

  std::ifstream t(PTXFilename);
  if (!t.is_open()) {
    std::cerr << "Error: cannot open " << PTXFilename << " for processing. Skipped.\n";
    exit(-1);
  }
  
  std::string ptx_str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

  // Create driver context
  checkCudaErrors(cuCtxCreate(&context, 0, device));

  // Create module for object
  checkCudaErrors(cuModuleLoadDataEx(&cudaModule, ptx_str.c_str(), 0, 0, 0));

  // Get kernel function
  checkCudaErrors(cuModuleGetFunction(&function, cudaModule, "GPU_Main"));

  // Device data
  CUdeviceptr devBufferInput;
  CUdeviceptr devInputSize;
  CUdeviceptr devPatterns;
  CUdeviceptr devBufferOutput;
  CUdeviceptr devStrides;

  int strideSize = GROUPTHREADS * sizeof(ulong) * 4;
  int strides = filesize/(strideSize * 2) + 1;
  int bufferSize = strides * strideSize;
  int outputSize = sizeof(ulong) * GROUPTHREADS * strides * (dist + 1) * GROUPBLOCKS;

  checkCudaErrors(cuMemAlloc(&devBufferInput, bufferSize));
  checkCudaErrors(cuMemAlloc(&devInputSize, sizeof(ulong)));
  checkCudaErrors(cuMemAlloc(&devPatterns, patternLen));
  checkCudaErrors(cuMemAlloc(&devBufferOutput, outputSize));
  // checkCudaErrors(cuMemsetD8(devBufferOutput, 0, outputSize));
  checkCudaErrors(cuMemAlloc(&devStrides, sizeof(int)));

  //Copy from host to device
  checkCudaErrors(cuMemcpyHtoD(devBufferInput, fileBuffer, bufferSize));
  checkCudaErrors(cuMemcpyHtoD(devInputSize, &filesize, sizeof(ulong)));
  checkCudaErrors(cuMemcpyHtoD(devPatterns, patternStr, patternLen));
  checkCudaErrors(cuMemcpyHtoD(devStrides, &strides, sizeof(int)));

  unsigned blockSizeX = GROUPTHREADS;
  unsigned blockSizeY = 1;
  unsigned blockSizeZ = 1;
  unsigned gridSizeX  = GROUPBLOCKS;
  unsigned gridSizeY  = 1;
  unsigned gridSizeZ  = 1;

  // Kernel parameters
  void *KernelParams[] = { &devBufferInput, &devInputSize, &devPatterns, &devBufferOutput, &devStrides};

  // std::cout << "Launching kernel\n";

  CUevent start;
  CUevent stop;
  float elapsedTime;

  cuEventCreate(&start, CU_EVENT_BLOCKING_SYNC);
  cuEventRecord(start,0);

  // Kernel launch
  checkCudaErrors(cuLaunchKernel(function, gridSizeX, gridSizeY, gridSizeZ,
                                 blockSizeX, blockSizeY, blockSizeZ,
                                 0, NULL, KernelParams, NULL));

  cuEventCreate(&stop, CU_EVENT_BLOCKING_SYNC);
  cuEventRecord(stop,0);
  cuEventSynchronize(stop);

  cuEventElapsedTime(&elapsedTime, start, stop);
  printf("Elapsed time : %f ms\n" ,elapsedTime);

  // Retrieve device data
  ulong * matchRslt = (ulong *) malloc(outputSize);
  checkCudaErrors(cuMemcpyDtoH(matchRslt, devBufferOutput, outputSize));


  // Clean-up
  checkCudaErrors(cuMemFree(devBufferInput));
  checkCudaErrors(cuMemFree(devInputSize));
  checkCudaErrors(cuMemFree(devBufferOutput));
  checkCudaErrors(cuMemFree(devPatterns));
  checkCudaErrors(cuMemFree(devStrides));
  checkCudaErrors(cuModuleUnload(cudaModule));
  checkCudaErrors(cuCtxDestroy(context));

  return matchRslt;
}