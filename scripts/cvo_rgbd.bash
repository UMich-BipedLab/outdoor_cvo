
cd build && make -j6 && cd .. && \

# fr1
#for i in freiburg1_desk_v freiburg1_desk2_v freiburg1_room_v freiburg1_360_v freiburg1_xyz_v freiburg1_rpy_v freiburg1_plant_v

# for i in freiburg1_desk
#fr2
# for i in

#fr3
for i in freiburg3_nostructure_texture_near freiburg3_structure_notexture_far freiburg3_structure_notexture_near freiburg3_structure_texture_far freiburg3_structure_texture_near freiburg3_nostructure_notexture_far freiburg3_nostructure_notexture_near

# all
#for i in freiburg1_desk freiburg1_desk2 freiburg1_room freiburg1_360 freiburg1_xyz freiburg1_teddy freiburg1_rpy freiburg1_plant freiburg1_floor freiburg3_nostructure_texture_far freiburg3_nostructure_texture_near freiburg3_structure_notexture_far freiburg3_structure_notexture_near freiburg3_structure_texture_far freiburg3_structure_texture_near freiburg3_nostructure_notexture_far freiburg3_nostructure_notexture_near
do
    echo ""
    echo "/********************** New Iteration *************************/"

    # gdb -ex run --args \
	    ./build/bin/cvo_align_gpu_rgbd /home/justin/data/tum/$i cvo_params/cvo_rgbd_params.yaml \
                                       cvo_rgbd_$i.txt 0 9999 
    
done
