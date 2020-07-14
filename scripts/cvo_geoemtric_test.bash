
cd build && make -j6 && cd .. && \
#for i in 05
for i in  00 01 02 03 04 05 06 07 08 09 10
do
    echo ""
    echo "/********************** New Iteration *************************/"

    #gdb -ex run --args \
	    ./build/bin/cvo_align_gpu_lidar_raw /home/justin/data/kitti/sequences/$i cvo_params/cvo_geometric_params_gpu.yaml \
                                       results/lidar_geometric_result/cvo_geometric_$i"_test.txt" 0 1000000
    
done
