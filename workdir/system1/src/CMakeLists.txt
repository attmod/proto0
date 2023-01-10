cmake_minimum_required(VERSION 3.12)

#add_subdirectory(stereo-vision)

# from robotology/segmentation
# add_subdirectory(blobExtractor)
# add_subdirectory(dispBlobber)
# add_subdirectory(edison)
# add_subdirectory(graphBased)
# add_subdirectory(lbpExtract)
# add_subdirectory(lumaChroma)
# add_subdirectory(seg2cloud)
# add_subdirectory(segmentation)

# Compiles, but broken on connect
# from robotology/logpolar
# add_subdirectory(logpolar)

# BROKEN
# from robotology/himrep
# add_subdirectory(caffeCoder)

add_subdirectory(image_colors)
add_subdirectory(image_colors_gpu)
add_subdirectory(image_edges)
add_subdirectory(image_blob)


add_subdirectory(micro_grasp)
add_subdirectory(micro_gaze)
add_subdirectory(micro_orientation)
add_subdirectory(micro_segment)
add_subdirectory(micro_actions)


add_subdirectory(gazebo_teleporter)
add_subdirectory(csv_logger)
add_subdirectory(image_forward)

add_subdirectory(micro_robot_reset)
