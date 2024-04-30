import os
import cv2
import numpy as np
import pcl

class MapGenerater:
    def __init__(self, input_pcd, dest_directory, resolution, map_width, map_height, min_points_in_pix, max_points_in_pix, min_height, max_height, **kwargs):
        self.input_pcd = input_pcd
        self.dest_directory = dest_directory
        self.resolution = resolution
        self.m2pix = 1.0 / resolution
        self.map_width = map_width
        self.map_height = map_height
        self.min_points_in_pix = min_points_in_pix
        self.max_points_in_pix = max_points_in_pix
        self.min_height = min_height
        self.max_height = max_height
        self.map = np.zeros((self.map_height, self.map_width), dtype=np.int32)

    def generate(self, cloud):
        self.map = np.zeros((self.map_height, self.map_width), dtype=np.int32)
      
        for point in cloud:
            if self.min_height <= point[2] <= self.max_height:
                x = int(point[0] * self.m2pix + self.map_width // 2)
                y = int(-point[1] * self.m2pix + self.map_width // 2)

                if (0 <= x < self.map_width) and (0 <= y < self.map_height):
                    self.map[y, x] += 1

        self.map = np.clip((self.map - self.min_points_in_pix) * -255.0 / (self.max_points_in_pix - self.min_points_in_pix) + 255, 0, 255).astype(np.uint8)
        return self.map
    
    def save_map(self, dest_directory):
        os.makedirs(dest_directory, exist_ok=True)

        cv2.imwrite(os.path.join(dest_directory, "map.png"), self.map)

        with open(os.path.join(dest_directory, "map.yaml"), "w") as f:
            f.write("image: map.png\n")
            f.write(f"resolution: {self.resolution}\n")
            f.write(f"origin: [{-self.resolution * self.map_width / 2}, {-self.resolution * self.map_height / 2}, 0.0]\n")
            f.write("occupied_thresh: 0.5\n")
            f.write("free_thresh: 0.2\n")
            f.write("negate: 0\n")
    
    def run(self):
        cloud = pcl.load(self.input_pcd)
        self.generate(cloud)
        self.save_map(self.dest_directory)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Generate 2d map from pointcloud(.pcd).', add_help=False)
    parser.add_argument('--help', action='help', default=argparse.SUPPRESS, help=argparse._('show this help message and exit'))
    parser.add_argument('input_pcd', metavar='filename.pcd', type=str, help='Input PCD file.')
    parser.add_argument('-d', '--dest_directory', type=str, default=os.getcwd(), help='Destination directory. defualt: current working directory')
    parser.add_argument('-r', '--resolution', type=float, default=0.1, help='Pixel resolution (meters / pix). defualt: 0.1')
    parser.add_argument('-w', '--map_width', type=int, default=1024, help='Map width [pix]. defualt: 1024')
    parser.add_argument('-h', '--map_height', type=int, default=1024, help='Map height [pix]. defualt: 1024')
    parser.add_argument('--min_points_in_pix', type=int, default=2, help='Min points in a occupied pix. defualt: 2')
    parser.add_argument('--max_points_in_pix', type=int, default=5, help='Max points in a pix for saturation. defualt: 5')
    parser.add_argument('--min_height', type=float, default=0.5, help='Min height of clipping range. defualt: 0.5')
    parser.add_argument('--max_height', type=float, default=1.0, help='Max height of clipping range. defualt: 1.0')

    args = parser.parse_args()

    print(f"input_pcd     :{args.input_pcd}")
    print(f"dest_directory:{args.dest_directory}")
    print(f"resolution    :{args.resolution}")

    generater = MapGenerater(**vars(args))
    generater.run()

    print("Finish converting point cloud to 2d map.")