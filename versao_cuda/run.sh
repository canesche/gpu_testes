nvcc -arch=sm_75 main.cu

BENCH=(32 64 128 192 256)

nvcc -ptx main.cu -o file.ptx

echo "" > res_gpu.csv
for ((i = 0; i < ${#BENCH[@]}; i++)); do
    echo "BENCH ${BENCH[i]}x${BENCH[i]}"
    echo "${BENCH[i]}x${BENCH[i]}" >> res_gpu.csv
    for ((j = 0; j < 4; j++)); do
        ./a.out ${BENCH[i]} ${BENCH[i]} >> res_gpu.csv
    done
    echo "\n" >> res_gpu.csv 
done
echo "done"