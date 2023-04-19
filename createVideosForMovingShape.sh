cd ~/shared/data/new_background/

mkdir videos_open_loop

read -p "Enter shape: " shape

cd ~/shared/data/new_background/videos_open_loop
mkdir $shape
cd ~/shared/data/new_background/videos_open_loop/$shape
mkdir trans_x_open_loop_moving_background
mkdir trans_y_open_loop_moving_background
mkdir rot_open_loop_moving_background
mkdir sc_open_loop_moving_background
mkdir 4dof_open_loop_moving_background

cd ~/shared/data/new_background/$shape/translation_x_1920x1080/
speeds_trans_x=(240 480 720 960 1200 1440)
for s in "${speeds_trans_x[@]}"
do
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/new_background/videos_open_loop/$shape/trans_x_open_loop_moving_background/"output_${s}_300Hz.mp4"
done

cd ~/shared/data/new_background/$shape/translation_y_1920x1080/
speeds_trans_y=(135 270 405 540 675 810)
for s in "${speeds_trans_y[@]}"
do
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/new_background/videos_open_loop/$shape/trans_y_open_loop_moving_background/"output_${s}_300Hz.mp4"
done

cd ~/shared/data/new_background/$shape/rotation_1920x1080/
speeds_rot=(100 150 175 200 225 250)
for s in "${speeds_rot[@]}"
do
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/new_background/videos_open_loop/$shape/rot_open_loop_moving_background/"output_${s}_300Hz.mp4"
done

cd ~/shared/data/new_background/$shape/scale_1920x1080/
speeds_sc=(100 200 300 400 600 800)
for s in "${speeds_sc[@]}"
do
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/new_background/videos_open_loop/$shape/sc_open_loop_moving_background/"output_${s}_300Hz.mp4"
done

cd ~/shared/data/new_background/$shape/combined_motions/
speeds_4dof=(100 200 300 400 600 800)
for s in "${speeds_4dof[@]}"
do
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/new_background/videos_open_loop/$shape/4dof_open_loop_moving_background/"output_${s}_300Hz.mp4"
done





