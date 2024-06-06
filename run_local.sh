#!/bin/bash
pathDatasetEuroc='/home/li/Datasets/EuRoc/' #Example, it is necesary to change it by the dataset path

# echo "Launching V102 with Monocular sensor"
# ./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH02 ./Examples/Monocular/EuRoC_TimeStamps/MH02.txt dataset-MH02_mono >z_log/ll_mono.log
# echo "------------------------------------"
# echo "Evaluation of V102 trajectory with Monocular-Inertial sensor"
# python evaluation/evaluate_ate_scale.py "$pathDatasetEuroc"MH02/mav0/state_groundtruth_estimate0/data.csv f_dataset-MH02_mono.txt --plot dataset-MH02_mono.pdf


# # Single Session Example (Visual-Inertial)
# echo "Launching V102 with Monocular-Inertial sensor"
# ./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH02 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH02.txt dataset-MH02_monoi >z_log/ll_monoi.log
# echo "------------------------------------"
# echo "Evaluation of V102 trajectory with Monocular-Inertial sensor"
# python evaluation/evaluate_ate_scale.py "$pathDatasetEuroc"MH02/mav0/state_groundtruth_estimate0/data.csv f_dataset-MH02_monoi.txt --plot dataset-MH02_monoi.pdf


# echo "Launching MH02 with Stereo sensor"
# ./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml "$pathDatasetEuroc"MH02 ./Examples/Stereo/EuRoC_TimeStamps/MH02.txt dataset-MH02_stereo >z_log/ll_stereo.log
# echo "------------------------------------"
# echo "Evaluation of V102 trajectory with Stereo sensor"
# python evaluation/evaluate_ate_scale.py "$pathDatasetEuroc"MH02/mav0/state_groundtruth_estimate0/data.csv f_dataset-MH02_stereo.txt --plot dataset-MH02_stereo.pdf


# echo "Launching MH02 with Stereo-Inertial sensor"
# ./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml "$pathDatasetEuroc"MH02 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH02.txt dataset-MH02_stereoi
# echo "------------------------------------"
# echo "Evaluation of V102 trajectory with Stereo sensor"
# python evaluation/evaluate_ate_scale.py "$pathDatasetEuroc"MH02/mav0/state_groundtruth_estimate0/data.csv f_dataset-MH02_stereoi.txt --plot dataset-MH02_stereoi.pdf


#https://blog.csdn.net/Welf688/article/details/124029171
#./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml /rgbd_dataset_freiburg1_desk rgbd_dataset_freiburg1_desk/associations.txt
#python associate.py /home/li/Datasets/EuRoc/rgbd_dataset_freiburg1_floor/rgb.txt /home/li/Datasets/EuRoc/rgbd_dataset_freiburg1_floor/depth.txt >/home/li/Datasets/EuRoc/rgbd_dataset_freiburg1_floor/associations.txt
#python associate.py /home/li/Datasets/EuRoc/rgbd_dataset_freiburg1_floor/associations.txt /home/li/Datasets/EuRoc/rgbd_dataset_freiburg1_floor/groundtruth.txt >/home/li/Datasets/EuRoc/rgbd_dataset_freiburg1_floor/associate_with_groundtruth


echo "Launching  with RGBD sensor"
./Examples/RGB-D/rgbd_tum ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM1.yaml "$pathDatasetEuroc"rgbd_dataset_freiburg1_floor  "$pathDatasetEuroc"rgbd_dataset_freiburg1_floor/associations.txt >z_log/ll_rgbd.log
echo "------------------------------------"
echo "Evaluation of V102 trajectory with RGBD sensor"


# python evaluation/evaluate_ate_scale.py "$pathDatasetEuroc"rgbd_dataset_freiburg1_floor/groundtruth.txt f_CameraTrajectory.txt --plot dataset-RGBD.pdf
