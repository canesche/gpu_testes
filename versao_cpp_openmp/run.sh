export OMP_NUM_THREADS=2

g++ main.cpp -fopenmp -O3

echo "" > res_gpu.csv
for ((j = 0; j < 4; j++)); do
./a.out >> res_gpu.csv
done

echo "done"
