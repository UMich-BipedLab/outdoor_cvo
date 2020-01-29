dataset="03"
file_name="cvo_f2f_tracking_"$dataset"_semantic.txt"

# file_name="cvo_kf_tracking.txt"

evo_traj kitti --ref /media/justin/LaCie/data/kitti/sequences/$dataset/groundtruth.txt  ~/research/outdoor_cvo/results/$file_name -p
