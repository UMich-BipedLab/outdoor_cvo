
cd build && make -j10 && cd .. && \
for i in 00 02

#for i in 00 01 02 03 04 05 07 08 09 10
do
    ./build/bin/cvo_align_gpu_lidar_semantic_raw /home/cel/data/kitti/sequences/$i cvo_params/cvo_semantic_params_gpu.yaml\
                                       cvo_semantic_$i.txt  0 2

done
