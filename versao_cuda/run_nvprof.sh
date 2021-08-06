nvcc main.cu

BENCH=(32 64 128 192 256)

nvcc -ptx main.cu -o file.ptx

for ((i = 0; i < ${#BENCH[@]}; i++)); do
    echo "BENCH ${BENCH[i]}x${BENCH[i]}"
    nvprof --metrics all ./a.out ${BENCH[i]} ${BENCH[i]} &> "metrics_"${BENCH[i]}".txt"
done
echo "done"