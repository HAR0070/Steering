import folium
import rosbag
from pyproj import Transformer

from scipy.spatial.transform import Rotation as R
from folium.plugins import PolyLineTextPath

chennai_map = folium.Map(location=[12.97, 80.22], zoom_start=20, max_zoom=25)#, max_zoom=22

# Create transformer: ECEF (EPSG:4978) → WGS84 geographic (EPSG:4326)
transformer = Transformer.from_crs("epsg:4978", "epsg:4326", always_xy=True)

bag = rosbag.Bag("/home/asl/Muni/datasets/vision_nav_data/new_sesor_steup.bag", allow_unindexed = True)

# for topic, msg, time_stamp in bag.read_messages(topics=['/fixposition/gnss1']):
#     folium.CircleMarker(location=(msg.latitude, msg.longitude), radius=1, fill=True).add_to(chennai_map)

# for topic, msg, time_stamp in bag.read_messages(topics=['/fixposition/gnss2']):
#     folium.CircleMarker(location=(msg.latitude, msg.longitude), radius=1, fill=True, color = "red").add_to(chennai_map)

points = []
yaws = []

for topic, msg, time_stamp in bag.read_messages(topics=['/imu_gps_synced_data']):
    # folium.CircleMarker(location=(msg.latitude, msg.longitude), radius=1, fill=True, color = "red").add_to(chennai_map)
    # print(msg)
    # exit(0)
    Pose = msg.odometry_ecef.pose.pose.position
    Orientation = msg.odometry_ecef.pose.pose.orientation

    # Convert from ECEF → Lat/Lon/Alt
    lon, lat, alt = transformer.transform(Pose.x, Pose.y, Pose.z)
    
    points.append([lat, lon])

    # Quaternion → Yaw
    r = R.from_quat([Orientation.x, Orientation.y, Orientation.z, Orientation.w])
    _, _, yaw = r.as_euler('xyz', degrees=True)
    yaws.append(yaw)

bag.close()

# ---- Plot Trajectory ----
# Option 1: draw small points
for lat, lon in points[::10]:  # plot every 10th point for speed
    folium.CircleMarker(location=[lat, lon], radius=1, color="red", fill=True).add_to(chennai_map)

# print(points)
# folium.PolyLine(points, color="blue", weight=2).add_to(chennai_map)


# Draw trajectory as blue line
path = folium.PolyLine(points, color="blue", weight=2.5).add_to(chennai_map)

# Add directional arrows along the path
PolyLineTextPath(
    path,
    '▶',  # Arrow symbol
    repeat=True,
    offset=5,
    attributes={'fill': 'red', 'font-weight': 'bold', 'font-size': '12'}
).add_to(chennai_map)

chennai_map.save("fp_osm.html")
