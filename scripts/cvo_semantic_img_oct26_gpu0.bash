export CUDA_VISIBLE_DEVICES=1
cd build && make -j6 && cd .. && \
for i in  04
#for i in 01  04 07 10 05 02 00 03 06 08 09 
#for i in 01 07  02 04 05 03    	
do
	echo "new seq $i"
    ./build/bin/cvo_align_gpu_semantic_img /home/v9999/media/seagate_2t/kitti/stereo/$i cvo_params/cvo_semantic_params_img_gpu0.yaml \
                                       cvo_img_semantic_jan11_$i.txt  0 20000

 	
done
