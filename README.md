# pointcloud_to_2dmap_ros

## Usage
- You must set input_pcd and dest_directory
```bash
roslaunch pointcloud_to_2dmap pointcloud_to_2dmap.launch input_pcd:=path/to/file.pcd dest_directory:=path/to/destination/directory/
```

## ros parameters
name | type | description
-----|------|-------------
resolution | double | Pixel resolution (meters / pix)
map_width | int | Map width [pix]
map_height | int | Map height [pix]
min_points_in_pix | int | Min points in a occupied pix
max_points_in_pix | int | Max points in a pix for saturation
min_height | double | Min height of clipping range
max_height | double | Max height of clipping range
input_pcd | str | Input PCD file
dest_directory | str | Destination directory

![Screenshot_20200716_160239](https://user-images.githubusercontent.com/31344317/87637926-e7adfc00-c77d-11ea-8987-19dffe614fa5.png)
