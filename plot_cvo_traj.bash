dataset="00"


# for dataset in 00 01 02 03 04 05 06 07 08 09 10
for dataset in 05
do
    file_name="cvo_f2f_tracking_"$dataset".txt"
    evo_traj kitti --ref /media/justin/LaCie/data/kitti/sequences/$dataset/groundtruth.txt  ~/research/outdoor_cvo/results/$file_name -p
done