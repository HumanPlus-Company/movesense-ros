# ms_image_view(rosbuild)
  This package is built for viewing the depth image of MoveSense.

## ms_image_view package 
3. Compiling the [ms_image_view] source codes
 
 **The package can not be compiled without ROS**
  **This package will be built with rosbuild**
  
		cd ms_image_view
		chmod +x build.sh
		./build.sh

  Executing these commands above will generate executable the  program**image_view**in *movesense-ros* folder.

4. Run the image view
  You should execute following command to add your package to ROS PATH:

		$ source ./build/devel/setup.bash
  
  You should execute following command in a new terminal before running the movesense-ros package:

		$ roscore
 
  Run image_view:
		
		$ rosrun ms_image_view image_view image:=/movesense_sensor/depth/image_raw 
  
  You can change the **Depth_MIN** and **Depth_Max** value to get different effects of depth image.
