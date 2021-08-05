#export OMP_NUM_THREADS=1

g++ main.cpp -fopenmp -O3

BENCH=(32 
    #64 128 192 
    #256
)

echo "" > res_gpu.csv
for ((i = 0; i < ${#BENCH[@]}; i++)); do
    echo "BENCH ${BENCH[i]}x${BENCH[i]}"
    echo "${BENCH[i]}x${BENCH[i]}" >> res_gpu.csv
    for ((j = 0; j < 10; j++)); do
        ./a.out #${BENCH[i]} ${BENCH[i]} >> res_gpu.csv
    done
    echo "\n" >> res_gpu.csv 
done
echo "done"
