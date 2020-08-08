
cd build && make -j6 && cd .. && \
for i in 04 
#for i in  00 01 02 03 04 05 06 07 08 09 10
do
    echo ""
    echo "/********************** New Iteration *************************/"

    gdb -ex run --args \
	    ./build/bin/cvo_align_gpu_lidar_raw_cov /home/justin/data/kitti/sequences/$i cvo_params/cvo_geometric_params_dense_kernel.yaml \
                                       cvo_geometric_cov_$i.txt 1 2
    
done
