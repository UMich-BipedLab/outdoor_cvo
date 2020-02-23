dataset="00"


# for dataset in 00 01 02 03 04 05 06 07 08 09 10
for dataset in 05
do
    file_name="cvo_f2f_tracking_"$dataset"_semantic.txt"
    evo_traj kitti --ref /media/justin/LaCie/data/kitti/sequences/$dataset/groundtruth.txt  ~/research/outdoor_cvo/results/$file_name -p
    evo_rpe kitti /media/justin/LaCie/data/kitti/sequences/$dataset/groundtruth.txt  ~/research/outdoor_cvo/results/$file_name -p
done

# evo_traj kitti --ref /media/justin/LaCie/data/kitti/sequences/04/groundtruth.txt  ~/research/outdoor_cvo/results/cvo_f2f_tracking_04_1.txt -p
#  evo_traj kitti --ref /media/justin/LaCie/data/kitti/sequences/04/groundtruth.txt  ~/research/outdoor_cvo/results/cvo_f2f_tracking_04_08.txt   ~/research/outdoor_cvo/results/cvo_f2f_tracking_04_04.txt ~/research/outdoor_cvo/results/cvo_f2f_tracking_04.txt -p --plot_mode=xz
#  evo_traj kitti --ref /media/justin/LaCie/data/kitti/sequences/04/groundtruth.txt  ~/research/outdoor_cvo/results/cvo_f2f_tracking_03_005.txt   ~/research/outdoor_cvo/results/cvo_f2f_tracking_03_04.txt ~/research/outdoor_cvo/results/cvo_f2f_tracking_03_08.txt -p --plot_mode=xz