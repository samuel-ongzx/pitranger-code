

def pure_pursuit(current_pose, start_pose, goal_pose):
  line = [start_pose.xy, goal_pose.xy]

  closest_pt = closest_point(current_pose.xy, line)

  g = closest_pt - current_pose.xy
  theta = np.arctan2(g[1],g[0])
  dh_fwd = nav.subtract_angles(current_pose.h, theta)

  R = np.linalg.norm(g) / (2*np.sin(dh_fwd))
  return [ROBOT_VELOCITY, ROBOT_VELOCITY / R]
