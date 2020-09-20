import rospy

# Not needed as the tf buffer essentially does this in the background, but the
# behaviour for custom caching of ROS data is implemented & functional...
# def cache_poses(data, existing_data):
#     if existing_data is None:
#         existing_data = {}
#     # Nested dictionairies for v[parent][child] = t ... joy
#     for t in data.transforms:
#         if t.header.frame_id not in existing_data:
#             existing_data[t.header.frame_id] = {}
#         existing_data[t.header.frame_id][t.child_frame_id] = t
#     return existing_data
