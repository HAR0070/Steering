import folium
from folium import Map, TileLayer
from pyproj import Transformer
import rosbag
from scipy.spatial.transform import Rotation as R

# Create folium map centered on Chennai
chennai_map = Map(location=[12.97, 80.22], zoom_start=20, max_zoom=40)#, max_zoom=25


# lyrs value	Description
# m	Standard roadmap (default)
# s	Satellite imagery
# y	Hybrid (satellite + labels)
# p	Terrain map
# h	Labels only (overlay useful with satellite)

# Add Google Maps base layer
google_tiles = TileLayer(
    tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
    attr='Google',
    name='Google Maps',
    overlay=False,
    control=True
)
google_tiles.add_to(chennai_map)

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

# (You can also use 'lyrs=s' for satellite, 'lyrs=p' for terrain)
# lyrs=m (normal), s (satellite), p (terrain), y (hybrid)

# Then add your trajectory as before
folium.PolyLine(points, color="blue", weight=2.5).add_to(chennai_map)

chennai_map.save("fp_google_sat.html")
