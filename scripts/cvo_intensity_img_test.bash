
cd build && make -j && cd .. && \
#for i in 04
for i in 05 06 07 
do
    ./build/bin/cvo_align_gpu_raw_img /home/rzh/media/sda1/ray/datasets/kitti/sequences/$i cvo_params/cvo_intensity_params_img.yaml \
                                       cvo_img_intensity_$i.txt 0 10000
 
done
