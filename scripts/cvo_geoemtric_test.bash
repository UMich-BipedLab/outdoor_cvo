
cd build && make -j6 && cd .. && \
#for i in 05
for i in 05 #00 01 02 03 04 05 06 07 08 09 10
do
    echo ""
    echo "/********************** Sequence "$i"*************************/"
    # for angle in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 60 90 120 150 180 210 240 270 300 330 331 332 333 334 335 336 337 338 339 340 341 342 343 344 345 346 347 348 349 350 351 352 353 354 355 356 357 358 359 360
    # for angle in -5 -4 -3 -2 -1 0 1 2 3 4 5
    # for frame in 1 2 3 4 5
    # do

    # gdb -ex run --args \
	    # ./build/bin/cvo_align_gpu_lidar_raw /home/cel/data/kitti/sequences/$i cvo_params/cvo_geometric_params_gpu.yaml \
        #                                results/lidar_geometric_result/cvo_geometric_$i"_indicator_evaluation_afterpointselection.txt" 0 2

        # mv inner_product_history.txt indicator_evaluation/$i"_afterpointselection_init_guess_inner_product_history.txt"
        # mv function_angle_history.txt indicator_evaluation/$i"_afterpointselection_init_guess_function_angle_history.txt"

	    # ./build/bin/cvo_align_gpu_lidar_raw /home/cel/data/kitti/sequences/$i cvo_params/cvo_geometric_params_gpu.yaml \
        #                                results/lidar_geometric_result/cvo_geometric_$i"_test_indicator.txt" $frame 1 #$angle #100000
        # mv indicator_evaluation/0_rotate_temp.pcd indicator_evaluation/raw_rotation/$i"_0_raw_rotation_"$angle"_degree.pcd"
        # mv indicator_evaluation/$frame"_rotate_temp.pcd" indicator_evaluation/raw_rotation/$i"_"$frame"_rawpointcloud.pcd"
        # mv indicator_evaluation/0_y_translate_4.pcd indicator_evaluation/$i"_0_y_translate_4.pcd"

                                       #cvo_align_gpu_lidar_loam

        ./build/bin/cvo_evaluate_indicator cvo_params/cvo_geometric_params_gpu.yaml \
                                            indicator_evaluation/$i"_evaluate_indicator_raw.txt" \
                                            $i
                                       
                                      
    # done
done
