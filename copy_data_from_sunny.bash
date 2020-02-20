# for seq in 01 02 03
# do
    # generating cvo points from images
    # echo "creating folder $seq"
    # mkdir /media/justin/LaCie/data/kitti/sequences/$seq
    # rm -r /media/justin/LaCie/data/kitti/sequences/$seq/cvo_points
    # mkdir /media/justin/LaCie/data/kitti/sequences/$seq/cvo_points
    # mkdir /media/justin/LaCie/data/kitti/sequences/$seq/cvo_points_pcd
    # mkdir /media/justin/LaCie/data/kitti/sequences/$seq/image_2
    # mkdir /media/justin/LaCie/data/kitti/sequences/$seq/image_3
    # mkdir /media/justin/LaCie/data/kitti/sequences/$seq/image_semantic
    # bash gen_kitti_pcd.bash $seq
    # sleep 10
    # scp -r $sunny_ip:/home/rzh/datasets/kitti/sequences/$seq/cvo_points /media/justin/LaCie/data/kitti/sequences/$seq
    # scp -r $sunny_ip:/home/rzh/datasets/kitti/sequences/$seq/image_2 /media/justin/LaCie/data/kitti/sequences/$seq
    # scp -r $sunny_ip:/home/rzh/datasets/kitti/sequences/$seq/image_3 /media/justin/LaCie/data/kitti/sequences/$seq
    # scp -r $sunny_ip:/home/rzh/datasets/kitti/sequences/$seq/cvo_calib.txt /media/justin/LaCie/data/kitti/sequences/$seq
    # scp -r $sunny_ip:/home/rzh/datasets/kitti/sequences/$seq/image_semantic /media/justin/LaCie/data/kitti/sequences/$seq
    # scp -r $sunny_ip:/home/rzh/datasets/kitti/sequences/$seq/groundtruth.txt /media/justin/LaCie/data/kitti/sequences/$seq
    # scp -r /media/justin/LaCie/data/kitti/sequences/$seq/groundtruth.txt $sunny_ip:/home/rzh/datasets/kitti/sequences/$seq/
    # run frame to frame cvo alignment
    # echo "./build/bin/cvo_test /media/justin/LaCie/data/kitti/sequences/$seq/cvo_points kitti_${seq}_out.txt 2000"
    # ./build/bin/cvo_test /media/justin/LaCie/data/kitti_$seq/cvo_points /media/justin/LaCie/data/kitti_$seq/kitti_${seq}_out.txt 2000

# done

# scp -r $sunny_ip:/home/rzh/datasets/kitti/sequences/05/image_semantic /media/justin/LaCie/data/kitti/sequences/05
# scp -r $sunny_ip:/home/rzh/datasets/kitti/sequences/06/image_semantic /media/justin/LaCie/data/kitti/sequences/06
# scp -r $sunny_ip:/home/rzh/datasets/kitti/sequences/07/image_semantic /media/justin/LaCie/data/kitti/sequences/07

# scp -r $sunny_ip:/home/tzuyuan/research/outdoor_cvo/results /home/justin/

# mkdir /home/justin/thinkpad_results/
scp -r $thinkpad_ip:~/code/outdoor_cvo/results /home/justin/thinkpad_results/
scp -r $thinkpad_ip:~/code/outdoor_cvo/cvo_params.txt /home/justin/thinkpad_results/