#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cassert>
#include "cuda.h"

#define GROUPTHREADS 64

void checkCudaErrors(CUresult err) {
  assert(err == CUDA_SUCCESS);
}

/// main - Program entry point
int RunPTX(std::string PTXFilename, char * fileBuffer, ulong filesize) {
  
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
    return 1;
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
  checkCudaErrors(cuModuleGetFunction(&function, cudaModule, "kernel"));

  // Device data
  CUdeviceptr devBufferInput;
  CUdeviceptr devBufferSize;
  CUdeviceptr devBufferOutput;

  int groupSize = GROUPTHREADS * sizeof(ulong) * 8;
  int bufferSize = (filesize/groupSize + 1) * groupSize;

  checkCudaErrors(cuMemAlloc(&devBufferInput, bufferSize));
  // checkCudaErrors(cuMemsetD8(devBufferInput, 0, bufferSize));
  checkCudaErrors(cuMemAlloc(&devBufferSize, sizeof(ulong)));
  checkCudaErrors(cuMemAlloc(&devBufferOutput, sizeof(ulong)*GROUPTHREADS));

  //Copy from host to device
  checkCudaErrors(cuMemcpyHtoD(devBufferInput, fileBuffer, bufferSize));
  checkCudaErrors(cuMemcpyHtoD(devBufferSize, &filesize, sizeof(ulong)));

  unsigned blockSizeX = GROUPTHREADS;
  unsigned blockSizeY = 1;
  unsigned blockSizeZ = 1;
  unsigned gridSizeX  = 1;
  unsigned gridSizeY  = 1;
  unsigned gridSizeZ  = 1;

  // Kernel parameters
  void *KernelParams[] = { &devBufferInput, &devBufferSize, &devBufferOutput};

  // std::cout << "Launching kernel\n";

  // Kernel launch
  checkCudaErrors(cuLaunchKernel(function, gridSizeX, gridSizeY, gridSizeZ,
                                 blockSizeX, blockSizeY, blockSizeZ,
                                 0, NULL, KernelParams, NULL));
  // std::cout << "kernel success.\n";
  // Retrieve device data
  ulong * matchCount = (ulong *) malloc(sizeof(ulong)*GROUPTHREADS);
  checkCudaErrors(cuMemcpyDtoH(matchCount, devBufferOutput, sizeof(ulong)*GROUPTHREADS));

  int count = 0;
  for (unsigned i = 0; i < GROUPTHREADS; ++i) {
    count += matchCount[i];
    // std::cout << i << ":" << matchCount[i] << "\n";
  }
  std::cout << count << "\n";

  // Clean-up
  checkCudaErrors(cuMemFree(devBufferInput));
  checkCudaErrors(cuMemFree(devBufferSize));
  checkCudaErrors(cuMemFree(devBufferOutput));
  checkCudaErrors(cuModuleUnload(cudaModule));
  checkCudaErrors(cuCtxDestroy(context));

  return 0;
}