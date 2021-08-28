//#----------------------------------------------------------------- CHECK CUDA
#define CHECK(call) { cudaAssert((call), __FILE__, __LINE__); }
inline void cudaAssert(cudaError_t code, const char* file, int line) {
	if (code != cudaSuccess) {
		cout << "\n\nCUDA ERROR: " << cudaGetErrorString(code)
			<< "\nFile: " << file << "\nLine: " << line << "\n\n";
		exit(code);
	}
}