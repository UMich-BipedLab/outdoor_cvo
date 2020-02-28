gt_dir=/home/cel/PERL/datasets/kitti_dataset/sequences/
gt_file_name=groundtruth.txt
results_dir=/home/cel/PERL/Algorithms/outdoor_cvo/current_result/

cd devkit/cpp
g++-9 -O3 -DNDEBUG -o evaluate_odometry evaluate_odometry.cpp matrix.cpp
cd ../..

for seq in 00 01 02 03 04 05 06 07 08 09 10
do
    echo $seq
    for file in $results_dir"cvo_f2f_tracking_"$seq*; 
    do
        results_file_name="${file##*/}"
        echo $results_file_name

        ./devkit/cpp/evaluate_odometry $seq $gt_dir $gt_file_name $results_dir $results_file_name
        # ./devkit/cpp/evaluate_odometry 05 /media/justin/LaCie/data/kitti/sequences/ groundtruth.txt ../../results/ cvo_f2f_tracking_05_08.txt
    done
done


