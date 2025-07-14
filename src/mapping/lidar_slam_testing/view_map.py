import argparse
import open3d as o3d

def view_map(map_path:str):
  global_map_o3d = o3d.io.read_point_cloud(map_path)
  o3d.visualization.draw_geometries([global_map_o3d])

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('map_path', help='path to the map to be viewed')
  args = parser.parse_args()
  map_path = str(args.map_path)
  view_map(map_path)