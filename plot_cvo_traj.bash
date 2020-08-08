# dataset="01"
# file_name="cvo_f2f_tracking_"$dataset"_08.txt"

# file_name="cvo_kf_tracking.txt"
for dataset in 04
do
    file_name="cvo_geometric_curv_"$dataset".txt"
    file_name2="cvo_geometric_"$dataset".txt"
    method="color_icp"
    icp_result_file="icp_"$method"_"$dataset".txt"

    evo_traj kitti --ref /media/curly_ssd_justin/DockerFolder/code/outdoor_cvo/ground_truth/$dataset.txt  /media/curly_ssd_justin/DockerFolder/code/outdoor_cvo/$file_name /media/curly_ssd_justin/DockerFolder/code/outdoor_cvo/results/$file_name2 /media/curly_ssd_justin/DockerFolder/code/outdoor_cvo/results/$file_name -p
    #  evo_traj kitti --ref /media/justin/LaCie/data/kitti/ground_truth/$dataset.txt /home/justin/research/outdoor_cvo/results/$file_name /home/justin/research/open3d_icp/results/$method/$icp_result_file -p
    # evo_traj kitti --ref /media/justin/LaCie/data/kitti/ground_truth/$dataset.txt /home/justin/research/open3d_icp/results/$method/$icp_result_file -p
    # evo_traj kitti --ref /media/justin/LaCie/data/kitti/ground_truth/$dataset.txt /home/justin/research/outdoor_cvo/results/$file_name /home/justin/research/outdoor_cvo/results/indicator_06/cvo_$dataset.txt /home/justin/research/outdoor_cvo/results/indicator_06_with_normal/cvo_normal_$dataset.txt   -p
    # evo_traj kitti --ref /media/justin/LaCie/data/kitti/ground_truth/$dataset.txt /home/justin/research/outdoor_cvo/results/$file_name /home/justin/research/outdoor_cvo/results/indicator_06/cvo_$dataset.txt   -p
    # evo_traj kitti --ref /media/justin/LaCie/data/kitti/ground_truth/$dataset.txt /home/justin/research/outdoor_cvo/results/indicator_06/cvo_$dataset.txt /home/justin/research/outdoor_cvo/results/indicator_06_with_normal/cvo_normal_$dataset.txt   -p
    # evo_traj kitti --ref /media/justin/LaCie/data/kitti/sequences/$dataset/groundtruth.txt  /home/justin/research/outdoor_cvo/results/$file_name -p
    # evo_rpe kitti -p /media/justin/LaCie/data/kitti/sequences/$dataset/groundtruth.txt  /home/justin/research/outdoor_cvo/results/$file_name
done
