cd ~/shared/data/moving_background/

mkdir videos_open_loop

read -p "Enter shape: " shape

cd ~/shared/data/moving_background/videos_open_loop
mkdir $shape
cd ~/shared/data/moving_background/videos_open_loop/$shape
mkdir trans_x_open_loop_moving_background
mkdir trans_y_open_loop_moving_background
mkdir rot_open_loop_moving_background
mkdir sc_open_loop_moving_background
mkdir 4dof_open_loop_moving_background

speeds_trans_x=(240 480 720 960 1200 1440)
speeds_trans_y=(135 270 405 540 675 810)
speeds_rot=(100 150 175 200 225 250)
speeds_sc=(100 200 300 400 600 800)
speeds_4dof=(100 200 300 400 600 800)

data_recording_time=20
n_frames=3072

for s in "${speeds_trans_x[@]}"
do
   let video_dur=$n_frames/$s
   let n_videos=$data_recording_time/$video_dur
   cd ~/shared/data/moving_background/$shape/translation_x_1920x1080/
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/moving_background/videos_open_loop/$shape/trans_x_open_loop_moving_background/"output_${s}_300Hz.mp4"
   echo $n_videos
   touch ~/shared/data/moving_background/videos_open_loop/$shape/trans_x_open_loop_moving_background/list.txt
   num=$(( $n_videos + 1 )) 
   cd ~/shared/data/moving_background/videos_open_loop/$shape/trans_x_open_loop_moving_background/
   exec 3>list.txt
   for ((i=1;i<=$num;i++)) 
   do
       echo file "output_${s}_300Hz.mp4" >&3
   done
   exec 3>&-
   if [ "$n_videos" -ne "0" ]; then
       ffmpeg -f concat -safe 0 -i list.txt -c copy ~/shared/data/moving_background/videos_open_loop/$shape/trans_x_open_loop_moving_background/"output_${s}_300Hz_merged.mp4"
       rm ~/shared/data/moving_background/videos_open_loop/$shape/trans_x_open_loop_moving_background/"output_${s}_300Hz.mp4"
       mv ~/shared/data/moving_background/videos_open_loop/$shape/trans_x_open_loop_moving_background/"output_${s}_300Hz_merged.mp4" ~/shared/data/moving_background/videos_open_loop/$shape/trans_x_open_loop_moving_background/"output_${s}_300Hz.mp4"
   fi
   sleep 1
done

rm ~/shared/data/moving_background/videos_open_loop/$shape/trans_x_open_loop_moving_background/list.txt

n_frames=1392
for s in "${speeds_trans_y[@]}"
do
   let video_dur=$n_frames/$s
   let n_videos=$data_recording_time/$video_dur
   cd ~/shared/data/moving_background/$shape/translation_y_1920x1080/
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/moving_background/videos_open_loop/$shape/trans_y_open_loop_moving_background/"output_${s}_300Hz.mp4"
   echo $n_videos
   touch ~/shared/data/moving_background/videos_open_loop/$shape/trans_y_open_loop_moving_background/list.txt
   num=$(( $n_videos + 1 )) 
   cd ~/shared/data/moving_background/videos_open_loop/$shape/trans_y_open_loop_moving_background/
   exec 3>list.txt
   for ((i=1;i<=$num;i++)) 
   do
       echo file "output_${s}_300Hz.mp4" >&3
   done
   if [ "$n_videos" -ne "0" ]; then
       ffmpeg -f concat -safe 0 -i list.txt -c copy ~/shared/data/moving_background/videos_open_loop/$shape/trans_y_open_loop_moving_background/"output_${s}_300Hz_merged.mp4"
       rm ~/shared/data/moving_background/videos_open_loop/$shape/trans_y_open_loop_moving_background/"output_${s}_300Hz.mp4"
       mv ~/shared/data/moving_background/videos_open_loop/$shape/trans_y_open_loop_moving_background/"output_${s}_300Hz_merged.mp4" ~/shared/data/moving_background/videos_open_loop/$shape/trans_y_open_loop_moving_background/"output_${s}_300Hz.mp4"
   fi
done

rm ~/shared/data/moving_background/videos_open_loop/$shape/trans_y_open_loop_moving_background/list.txt

n_frames=2200
for s in "${speeds_sc[@]}"
do
   let video_dur=$n_frames/$s
   let n_videos=$data_recording_time/$video_dur
   cd ~/shared/data/moving_background/$shape/scale_1920x1080/
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/moving_background/videos_open_loop/$shape/sc_open_loop_moving_background/"output_${s}_300Hz.mp4"
   echo $n_videos
   touch ~/shared/data/moving_background/videos_open_loop/$shape/sc_open_loop_moving_background/list.txt
   num=$(( $n_videos + 1 )) 
   cd ~/shared/data/moving_background/videos_open_loop/$shape/sc_open_loop_moving_background/
   exec 3>list.txt
   for ((i=1;i<=$num;i++)) 
   do
       echo file "output_${s}_300Hz.mp4" >&3
   done
   if [ "$n_videos" -ne "0" ]; then
       ffmpeg -f concat -safe 0 -i list.txt -c copy ~/shared/data/moving_background/videos_open_loop/$shape/sc_open_loop_moving_background/"output_${s}_300Hz_merged.mp4"
       rm ~/shared/data/moving_background/videos_open_loop/$shape/sc_open_loop_moving_background/"output_${s}_300Hz.mp4"
       mv ~/shared/data/moving_background/videos_open_loop/$shape/sc_open_loop_moving_background/"output_${s}_300Hz_merged.mp4" ~/shared/data/moving_background/videos_open_loop/$shape/sc_open_loop_moving_background/"output_${s}_300Hz.mp4"
   fi
done

rm ~/shared/data/moving_background/videos_open_loop/$shape/sc_open_loop_moving_background/list.txt

n_frames=1216

for s in "${speeds_rot[@]}"
do
   let video_dur=$n_frames/$s
   let n_videos=$data_recording_time/$video_dur
   cd ~/shared/data/moving_background/$shape/rotation_1920x1080/
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/moving_background/videos_open_loop/$shape/rot_open_loop_moving_background/"output_${s}_300Hz.mp4"
   echo $n_videos
   touch ~/shared/data/moving_background/videos_open_loop/$shape/rot_open_loop_moving_background/list.txt
   num=$(( $n_videos + 1 )) 
   cd ~/shared/data/moving_background/videos_open_loop/$shape/rot_open_loop_moving_background/
   exec 3>list.txt
   for ((i=1;i<=$num;i++)) 
   do
       echo file "output_${s}_300Hz.mp4" >&3
   done
   exec 3>&-
   if [ "$n_videos" -ne "0" ]; then
       ffmpeg -f concat -safe 0 -i list.txt -c copy ~/shared/data/moving_background/videos_open_loop/$shape/rot_open_loop_moving_background/"output_${s}_300Hz_merged.mp4"
       rm ~/shared/data/moving_background/videos_open_loop/$shape/rot_open_loop_moving_background/"output_${s}_300Hz.mp4"
       mv ~/shared/data/moving_background/videos_open_loop/$shape/rot_open_loop_moving_background/"output_${s}_300Hz_merged.mp4" ~/shared/data/moving_background/videos_open_loop/$shape/rot_open_loop_moving_background/"output_${s}_300Hz.mp4"
   fi
   sleep 1
done

rm ~/shared/data/moving_background/videos_open_loop/$shape/rot_open_loop_moving_background/list.txt

n_frames=2251
for s in "${speeds_4dof[@]}"
do
   let video_dur=$n_frames/$s
   let n_videos=$data_recording_time/$video_dur
   cd ~/shared/data/moving_background/$shape/combined_motions/
   ffmpeg -framerate $s -i image-%05d.jpg -vc h264 -pix_fmt yuv420p -crf 10 -r 300 ~/shared/data/moving_background/videos_open_loop/$shape/4dof_open_loop_moving_background/"output_${s}_300Hz.mp4"
   echo $n_videos
   touch ~/shared/data/moving_background/videos_open_loop/$shape/4dof_open_loop_moving_background/list.txt
   num=$(( $n_videos + 1 )) 
   cd ~/shared/data/moving_background/videos_open_loop/$shape/4dof_open_loop_moving_background/
   exec 3>list.txt
   for ((i=1;i<=$num;i++)) 
   do
       echo file "output_${s}_300Hz.mp4" >&3
   done
   if [ "$n_videos" -ne "0" ]; then
       ffmpeg -f concat -safe 0 -i list.txt -c copy ~/shared/data/moving_background/videos_open_loop/$shape/4dof_open_loop_moving_background/"output_${s}_300Hz_merged.mp4"
       rm ~/shared/data/moving_background/videos_open_loop/$shape/4dof_open_loop_moving_background/"output_${s}_300Hz.mp4"
       mv ~/shared/data/moving_background/videos_open_loop/$shape/4dof_open_loop_moving_background/"output_${s}_300Hz_merged.mp4" ~/shared/data/moving_background/videos_open_loop/$shape/4dof_open_loop_moving_background/"output_${s}_300Hz.mp4"
   fi
done

rm ~/shared/data/moving_background/videos_open_loop/$shape/4dof_open_loop_moving_background/list.txt


