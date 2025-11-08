#include "plan_env/sdf_map.h"

// void SDFmap::odomCallback(const carstatemsgs::CarState::ConstPtr &msg){
//   odom_ = *msg;
//   has_odom_ = true;
// }

void SDFmap::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  static tf2_ros::Buffer tf_buffer(ros::Duration(10.0));
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  // Get transformation information
  geometry_msgs::TransformStamped transformStamped;
  try {
      transformStamped = tf_buffer.lookupTransform("world", msg->header.frame_id,
                                                  msg->header.stamp, ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
  }

  odom_pos_.head(2) = Eigen::Vector2d(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
  odom_pos_[2] = tf::getYaw(transformStamped.transform.rotation);

  // Recenter local map window around robot if needed
  slideWindowIfNeeded();

  // Perform coordinate transformation
  sensor_msgs::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, transformStamped);

  // Convert to PCL format
  pcl::fromROSMsg(transformed_cloud, cloud_);
  
  occ_need_update_ = true;
}

void SDFmap::updateOccupancyCallback(const ros::TimerEvent& /*event*/){
  if(!occ_need_update_) return;

  if(!if_perspective_){
    ros::Time t1, t2;
    t1 = ros::Time::now();

    x_lower_ = std::max(odom_pos_.x() - ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_lower_);
    x_upper_ = std::min(odom_pos_.x() + ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_upper_);
    y_lower_ = std::max(odom_pos_.y() - ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_lower_);
    y_upper_ = std::min(odom_pos_.y() + ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_upper_);

    X_SIZE_ = ceil((x_upper_ - x_lower_) / grid_interval_);
    Y_SIZE_ = ceil((y_upper_ - y_lower_) / grid_interval_);
    XY_SIZE_ = X_SIZE_ * Y_SIZE_;

    raycastProcess();

    if(if_cirSupRaycast_){
      static int cirSup = 1;
      cirSup ++;
      if(cirSup%3==0){
        cirSupRaycastProcess();
        cirSup = 1;
      }
    }

    RemoveOutliers();
    
    // Update the map based on occupancy_map_
    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

    // // Will treat obstacles as free space
    // for (int x=min_id.x(); x<=max_id.x(); x++) {
    //   for (int y=min_id.y(); y<=max_id.y(); y++){
    //     if(occupancy_map_[Index2Vectornum(x,y)] >= clamp_min_log_ && occupancy_map_[Index2Vectornum(x,y)] <= min_occupancy_log_){
    //       gridmap_[Index2Vectornum(x,y)] = Unoccupied;
    //     }
    //     else if(occupancy_map_[Index2Vectornum(x,y)] > min_occupancy_log_){
    //       gridmap_[Index2Vectornum(x,y)] = Occupied;
    //     }
    //   }
    // }
    // Will not treat obstacles as free space!!
    for (int x=min_id.x(); x<=max_id.x(); x++) {
      for (int y=min_id.y(); y<=max_id.y(); y++){
        int vecIndex = Index2Vectornum(x,y);
        if(gridmap_[vecIndex] == Unknown && occupancy_map_[vecIndex] >= clamp_min_log_ && occupancy_map_[vecIndex] <= min_occupancy_log_){
          gridmap_[vecIndex] = Unoccupied;
        }
        else if(occupancy_map_[vecIndex] > min_occupancy_log_){
          gridmap_[vecIndex] = Occupied;
        }
      }
    }
  }

  
  else{
    x_lower_ = std::max(odom_pos_.x() - ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_lower_);
    x_upper_ = std::min(odom_pos_.x() + ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_upper_);
    y_lower_ = std::max(odom_pos_.y() - ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_lower_);
    y_upper_ = std::min(odom_pos_.y() + ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_upper_);

    X_SIZE_ = ceil((x_upper_ - x_lower_) / grid_interval_);
    Y_SIZE_ = ceil((y_upper_ - y_lower_) / grid_interval_);
    XY_SIZE_ = X_SIZE_ * Y_SIZE_;

    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

    for (int x=min_id.x(); x<=max_id.x(); x++) {
      for (int y=min_id.y(); y<=max_id.y(); y++){
        if(gridmap_[Index2Vectornum(x,y)] == Unknown)
          gridmap_[Index2Vectornum(x,y)] = Unoccupied;
      }
    }

    for(auto point:cloud_.points){
      Eigen::Vector2d coord = Eigen::Vector2d(point.x, point.y);
      if(!isInGloMap(coord)){
        continue;
      }
      Eigen::Vector2i idx = coord2gridIndex(coord);
      gridmap_[Index2Vectornum(idx)] = Occupied;
    }
  }


  has_map_ = true;
  esdf_need_update_ = true;
  occ_need_update_ = false;
}

void SDFmap::raycastProcess(){
  int points_cnt = cloud_.points.size();
  
  update_odom_ = odom_pos_.head(2);
  Eigen::Vector2d cur_point;
  int vox_idx;
  double length;

  Eigen::Vector3d ray_pt;
  Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);

  RayCaster raycaster;

  for (int i = 0; i < points_cnt; ++i) {
    cur_point << cloud_.points[i].x, cloud_.points[i].y;
    if(!isInGloMap(cur_point)){
      cur_point = closetPointInMap(cur_point, update_odom_);
      length = (cur_point - update_odom_).norm();
      if(length > detection_range_){
        cur_point = (cur_point - update_odom_) / length * detection_range_ + update_odom_;
      }
      vox_idx = setCacheOccupancy(cur_point, 0);
    }
    else {
      length = (cur_point - update_odom_).norm();
      if (length > detection_range_) {
        cur_point = (cur_point - update_odom_) / length * detection_range_ + update_odom_;
        vox_idx = setCacheOccupancy(cur_point, 0);
      } else {
        vox_idx = setCacheOccupancy(cur_point, 1);
      }
    }
    std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(coord2gridIndex(update_odom_), coord2gridIndex(cur_point));
    
    int size = line.size() - 1;

    for (int i=0; i<size; i++) {
      vox_idx = setCacheOccupancy(line[i], 0);
    }

  }

  updateOccupancyMap();
}

// Add a circle to prevent no points, resulting in no safe position points
void SDFmap::cirSupRaycastProcess(){
  Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);
  double length;
  int vox_idx;

  std::vector<Eigen::Vector2d> cir_points;
  for(double x = odom_pos_.x() - detection_range_; x < odom_pos_.x() + detection_range_+1e-10;  x += 2*detection_range_){
    for(double y = odom_pos_.y() - detection_range_; y < odom_pos_.y() + detection_range_+1e-10;  y += grid_interval_){
      cir_points.emplace_back(x,y);
    }
  }
  for(double y = odom_pos_.y() - detection_range_; y < odom_pos_.y() + detection_range_+1e-10;  y += 2*detection_range_){
    for(double x = odom_pos_.x() - detection_range_; x < odom_pos_.x() + detection_range_+1e-10;  x += grid_interval_){
      cir_points.emplace_back(x,y);
    }
  }

  RayCaster raycaster;

  for(auto cir_point:cir_points){
   
    if(hrz_limited_){
      double angle = atan2(cir_point.y() - odom_pos_.y(), cir_point.x() - odom_pos_.x());
      angle = normalize_angle(angle - odom_pos_.z());
      if(angle < -hrz_laser_range_dgr_/2.2 || angle >hrz_laser_range_dgr_/2.2){
        continue;
      }
    }

    if(!isInGloMap(cir_point)){
      cir_point = closetPointInMap(cir_point, update_odom_);
    }

    length = (cir_point - update_odom_).norm();
    if(length > detection_range_){
      cir_point = (cir_point - update_odom_) / length * detection_range_ + update_odom_;
    }

    std::vector<Eigen::Vector2i> line;
    raycaster.setInput(Eigen::Vector3d(cir_point.x(), cir_point.y(), 0.1) / grid_interval_, Eigen::Vector3d(update_odom_.x(), update_odom_.y(), 0.1) / grid_interval_);

    bool occ = false;
    Eigen::Vector3d ray_pt;
    while (raycaster.step(ray_pt)) {
      Eigen::Vector2d tmp = (ray_pt.head(2) + half) * grid_interval_;
      Eigen::Vector2i tmp_idx = coord2gridIndex(tmp);
      line.emplace_back(tmp_idx);
      if( gridmap_[Index2Vectornum(tmp_idx)] == Occupied ||
          (tmp_idx.y() < GLY_SIZE_-1 && gridmap_[Index2Vectornum(tmp_idx.x(), tmp_idx.y()+1)] == Occupied) ||
          (tmp_idx.y() > 0 && gridmap_[Index2Vectornum(tmp_idx.x(), tmp_idx.y()-1)] == Occupied) ||
          (tmp_idx.x() < GLX_SIZE_-1 && gridmap_[Index2Vectornum(tmp_idx.x()+1, tmp_idx.y())] == Occupied) ||
          (tmp_idx.x() > 0 && gridmap_[Index2Vectornum(tmp_idx.x()-1, tmp_idx.y())] == Occupied) ){
        occ = true;
        break;
      }
    }
    if(occ) continue;
    
    int size = line.size()-1;
    for (int i=0; i<size; i++) {
      vox_idx = setCacheOccupancy(line[i], 0);
    }
  }

  // updateOccupancyMap();

  Eigen::Vector2i min_id, max_id;
  min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
  max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

  while (!cache_voxel_.empty()) {
    Eigen::Vector2i idx = cache_voxel_.front();
    int idx_ctns = Index2Vectornum(idx);
    cache_voxel_.pop();

    double log_odds_update =
      count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 4*count_hit_[idx_ctns] ?
      prob_hit_log_ :
      prob_miss_log_;

    log_odds_update = 0.0;

    count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
    if (!in_local) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
    }

    occupancy_map_[idx_ctns] =
        std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                clamp_max_log_);
  }

}

void SDFmap::updateOccupancyMap(){
  Eigen::Vector2i min_id, max_id;
  min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
  max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

  while (!cache_voxel_.empty()) {
    Eigen::Vector2i idx = cache_voxel_.front();
    int idx_ctns = Index2Vectornum(idx);
    cache_voxel_.pop();

    double log_odds_update =
      count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 3*count_hit_[idx_ctns] ?
      prob_hit_log_ :
      prob_miss_log_;

    count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
    if (!in_local) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
    }

    occupancy_map_[idx_ctns] =
        std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                clamp_max_log_);
  }
}

void SDFmap::RemoveOutliers(){
  std::vector<Eigen::Vector2d> cir_points;
  for(double x = odom_pos_.x() - detection_range_; x < odom_pos_.x() + detection_range_+1e-10;  x += grid_interval_)
    for(double y = odom_pos_.y() - detection_range_; y < odom_pos_.y() + detection_range_+1e-10;  y += grid_interval_)
      cir_points.emplace_back(x,y);
  double xlow = global_x_lower_ + grid_interval_, xup = global_x_upper_ - grid_interval_;
  double ylow = global_y_lower_ + grid_interval_, yup = global_y_upper_ - grid_interval_;
  for(auto &cir_point:cir_points){
    if(cir_point.x() > xlow && cir_point.x() < xup && cir_point.y() > ylow && cir_point.y() < yup){
      Eigen::Vector2i cidx = coord2gridIndex(cir_point);
      if(gridmap_[Index2Vectornum(cidx)] == Unknown){
        if( (cidx.y() < GLY_SIZE_-1 && gridmap_[Index2Vectornum(cidx.x(), cidx.y()+1)] == Unoccupied) &&
            (cidx.y() > 0 && gridmap_[Index2Vectornum(cidx.x(), cidx.y()-1)] == Unoccupied) &&
            (cidx.x() < GLX_SIZE_-1 && gridmap_[Index2Vectornum(cidx.x()+1, cidx.y())] == Unoccupied) &&
            (cidx.x() > 0 && gridmap_[Index2Vectornum(cidx.x()-1, cidx.y())] == Unoccupied) ){
          gridmap_[Index2Vectornum(cidx)] = Unoccupied;
        }
      }
    }
  }
  Eigen::Vector2i idx = coord2gridIndex(odom_pos_.head(2));
  for(int i=-1;i<=1;i++) for(int j=-1;j<=1;j++){
    int nx = idx.x()+i, ny = idx.y()+j;
    if(nx>=0 && nx<GLX_SIZE_ && ny>=0 && ny<GLY_SIZE_){
      if(gridmap_[Index2Vectornum(nx, ny)] == Unknown) gridmap_[Index2Vectornum(nx, ny)] = Unoccupied;
    }
  }
}

int SDFmap::setCacheOccupancy(Eigen::Vector2d pos, int occ) {
  if (occ != 1 && occ != 0) return -1;

  Eigen::Vector2i idx = coord2gridIndex(pos);
  int idx_ctns = Index2Vectornum(idx);

  count_hit_and_miss_[idx_ctns] += 1;

  if (count_hit_and_miss_[idx_ctns] == 1) {
    cache_voxel_.push(idx);
  }

  if (occ == 1) count_hit_[idx_ctns] += 1;

  return idx_ctns;
}


int SDFmap::setCacheOccupancy(Eigen::Vector2i idx, int occ) {
  if (occ != 1 && occ != 0) return -1;

  int idx_ctns = Index2Vectornum(idx);

  count_hit_and_miss_[idx_ctns] += 1;

  if (count_hit_and_miss_[idx_ctns] == 1) {
    cache_voxel_.push(idx);
  }

  if (occ == 1) count_hit_[idx_ctns] += 1;

  return idx_ctns;
}


std::vector<Eigen::Vector2i> SDFmap::getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end){
    std::vector<Eigen::Vector2i> line;
    
    int dx = abs(end.x() - start.x());
    int dy = abs(end.y() - start.y());
    int sx = (start.x() < end.x()) ? 1 : -1;
    int sy = (start.y() < end.y()) ? 1 : -1;
    int err = dx - dy;

    double x0 = start.x();
    double y0 = start.y();

    while (true) {
        line.emplace_back(x0, y0);
        if (x0 == end.x() && y0 == end.y()) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    return line;
}

void SDFmap::updateESDFCallback(const ros::TimerEvent& /*event*/){
  if(!esdf_need_update_) return;
  updateESDF2d();

  has_esdf_ = true;
  esdf_need_update_ = false;
  
}

void SDFmap::visCallback(const ros::TimerEvent& /*event*/){
  if(has_map_){
    publish_gridmap();
    publishLocalBounds();
  }
  if(has_esdf_){
    publish_ESDF();
  }
}

void SDFmap::publishLocalBounds(){
  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker m;
  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();
  m.ns = "local_bounds";
  m.id = 0;
  m.type = visualization_msgs::Marker::CUBE;
  m.action = visualization_msgs::Marker::ADD;
  // center of window
  double cx = (global_x_lower_ + global_x_upper_) * 0.5;
  double cy = (global_y_lower_ + global_y_upper_) * 0.5;
  m.pose.position.x = cx;
  m.pose.position.y = cy;
  m.pose.position.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = (global_x_upper_ - global_x_lower_);
  m.scale.y = (global_y_upper_ - global_y_lower_);
  m.scale.z = 0.01;
  m.color.r = 0.0f; m.color.g = 0.5f; m.color.b = 1.0f; m.color.a = 0.2f;
  m.frame_locked = true;
  arr.markers.push_back(m);
  debug_maker.publish(arr);
}

inline void SDFmap::grid_insertbox(Eigen::Vector3d location,Eigen::Matrix3d euler,Eigen::Vector3d size){
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0);
  Eigen::Vector3d z(0,0,1);
  x  = euler*x;
  y  = euler*y;
  z  = euler*z;

  float insert_interval = 0.5;
  for(float i=-size.x()/2;i<=size.x()/2;i+=grid_interval_*insert_interval)
    for(float j=-size.y()/2;j<=size.y()/2;j+=grid_interval_*insert_interval)
      for(float k=-size.z()/2;k<=size.z()/2;k+=grid_interval_*insert_interval){
        Eigen::Vector3d point = location+i*x+j*y+k*z;
        setObs(point);
      }
}

Eigen::Vector2d SDFmap::gridIndex2coordd(const Eigen::Vector2i &index){
  Eigen::Vector2d pt;
  pt(0) = ((double)index(0) + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)index(1) + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

Eigen::Vector2d SDFmap::gridIndex2coordd(const int &x, const int &y){
  Eigen::Vector2d pt;
  pt(0) = ((double)x + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)y + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

Eigen::Vector2i SDFmap::coord2gridIndex(const Eigen::Vector2d &pt){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((pt(0) - global_x_lower_) * inv_grid_interval_), 0), GLX_SIZE_ - 1),
      std::min(std::max(int((pt(1) - global_y_lower_) * inv_grid_interval_), 0), GLY_SIZE_ - 1);
  return idx;
}

void SDFmap::setObs(const Eigen::Vector3d coord){
  float coord_x = coord.x(), coord_y = coord.y();
  if (coord_x < global_x_lower_ || coord_y < global_y_lower_ || coord_x >= global_x_upper_ || coord_y >= global_y_upper_) return;
  int idx_x = static_cast<int>((coord_x - global_x_lower_) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - global_y_lower_) * inv_grid_interval_);
  gridmap_[Index2Vectornum(idx_x, idx_y)] = Occupied;
}

void SDFmap::setObs(const Eigen::Vector2d coord){
  float coord_x = coord.x(), coord_y = coord.y();
  if (coord_x < global_x_lower_ || coord_y < global_y_lower_ || coord_x >= global_x_upper_ || coord_y >= global_y_upper_) return;
  int idx_x = static_cast<int>((coord_x - global_x_lower_) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - global_y_lower_) * inv_grid_interval_);
  gridmap_[Index2Vectornum(idx_x, idx_y)] = Occupied;
}

int SDFmap::Index2Vectornum(const int &x, const int &y){
  // logical (x,y) to physical with ring buffer offsets
  int sx = (x + buf_start_x_) % GLX_SIZE_;
  if(sx < 0) sx += GLX_SIZE_;
  int sy = (y + buf_start_y_) % GLY_SIZE_;
  if(sy < 0) sy += GLY_SIZE_;
  return sx * GLY_SIZE_ + sy;
}

int SDFmap::Index2Vectornum(const Eigen::Vector2i &index){
  return Index2Vectornum(index.x(), index.y());
}

Eigen::Vector2i SDFmap::vectornum2gridIndex(const int &num){
  // physical num to logical (x,y)
  int phys_x = num / GLY_SIZE_;
  int phys_y = num % GLY_SIZE_;
  int lx = (phys_x - buf_start_x_);
  if(lx < 0) lx += GLX_SIZE_;
  int ly = (phys_y - buf_start_y_);
  if(ly < 0) ly += GLY_SIZE_;
  return Eigen::Vector2i(lx, ly);
}

void SDFmap::slideWindowIfNeeded(){
  if(!dynamic_local_map_) return;
  Eigen::Vector2d robot = odom_pos_.head(2);
  if(!map_origin_initialized_){
    map_origin_world_ = robot - Eigen::Vector2d(half_size_m_, half_size_m_);
    global_x_lower_ = map_origin_world_.x();
    global_x_upper_ = map_origin_world_.x() + local_map_size_m_;
    global_y_lower_ = map_origin_world_.y();
    global_y_upper_ = map_origin_world_.y() + local_map_size_m_;
    map_origin_initialized_ = true;
    // full clear once at init
    std::fill(distance_buffer_all_.begin(), distance_buffer_all_.end(), std::numeric_limits<double>::max());
    memset(gridmap_, Unknown, GLXY_SIZE_ * sizeof(uint8_t));
    std::fill(occupancy_map_.begin(), occupancy_map_.end(), clamp_min_log_ - unknown_flag_);
    std::fill(count_hit_.begin(), count_hit_.end(), 0);
    std::fill(count_hit_and_miss_.begin(), count_hit_and_miss_.end(), 0);
    esdf_need_update_ = true;
    return;
  }
  // compute desired origin centered on robot
  Eigen::Vector2d desired_origin = robot - Eigen::Vector2d(half_size_m_, half_size_m_);
  // compute integer-cell shift
  int shift_x = static_cast<int>(floor((desired_origin.x() - global_x_lower_) * inv_grid_interval_));
  int shift_y = static_cast<int>(floor((desired_origin.y() - global_y_lower_) * inv_grid_interval_));
  if(shift_x == 0 && shift_y == 0) return;

  // update world bounds to new origin aligned by grid
  global_x_lower_ += shift_x * grid_interval_;
  global_x_upper_ = global_x_lower_ + local_map_size_m_;
  global_y_lower_ += shift_y * grid_interval_;
  global_y_upper_ = global_y_lower_ + local_map_size_m_;

  // advance ring offsets (logical->physical start)
  buf_start_x_ = (buf_start_x_ + shift_x) % GLX_SIZE_;
  if(buf_start_x_ < 0) buf_start_x_ += GLX_SIZE_;
  buf_start_y_ = (buf_start_y_ + shift_y) % GLY_SIZE_;
  if(buf_start_y_ < 0) buf_start_y_ += GLY_SIZE_;

  // initialize newly entered columns/rows for gridmap_, occupancy_map_, distance & counters
  auto init_col = [&](int lx){
    for(int ly=0; ly<GLY_SIZE_; ++ly){
      int v = Index2Vectornum(lx, ly);
      gridmap_[v] = Unknown;
      occupancy_map_[v] = clamp_min_log_ - unknown_flag_;
      distance_buffer_all_[v] = std::numeric_limits<double>::max();
      count_hit_[v] = 0; count_hit_and_miss_[v] = 0;
    }
  };
  auto init_row = [&](int ly){
    for(int lx=0; lx<GLX_SIZE_; ++lx){
      int v = Index2Vectornum(lx, ly);
      gridmap_[v] = Unknown;
      occupancy_map_[v] = clamp_min_log_ - unknown_flag_;
      distance_buffer_all_[v] = std::numeric_limits<double>::max();
      count_hit_[v] = 0; count_hit_and_miss_[v] = 0;
    }
  };

  // initialize columns entering from +x or -x side
  if(shift_x != 0){
    if(shift_x > 0){
      // shift right: new columns appear on logical [GLX_SIZE_-shift_x, GLX_SIZE_-1]
      for(int dx=0; dx<shift_x; ++dx){ init_col(GLX_SIZE_-1 - dx); }
    }else{
      // shift left: new columns appear on logical [0, -shift_x-1]
      for(int dx=0; dx<-shift_x; ++dx){ init_col(dx); }
    }
  }
  // initialize rows entering from +y or -y side
  if(shift_y != 0){
    if(shift_y > 0){
      for(int dy=0; dy<shift_y; ++dy){ init_row(GLY_SIZE_-1 - dy); }
    }else{
      for(int dy=0; dy<-shift_y; ++dy){ init_row(dy); }
    }
  }

  esdf_need_update_ = true;
}

void SDFmap::updateESDF2d(){
  Eigen::Vector2i min_esdf(floor(std::max(0.0, odom_pos_.x() - detection_range_ - global_x_lower_)*inv_grid_interval_), floor(std::max(0.0, odom_pos_.y() - detection_range_ - global_y_lower_)*inv_grid_interval_));
  Eigen::Vector2i max_esdf(ceil(std::min(global_x_upper_ - global_x_lower_, odom_pos_.x() + detection_range_ - global_x_lower_)*inv_grid_interval_) - 1,
                           ceil(std::min(global_y_upper_ - global_y_lower_, odom_pos_.y() + detection_range_ - global_y_lower_)*inv_grid_interval_) - 1);
  int update_X_SIZE = max_esdf.x() - min_esdf.x();
  int update_Y_SIZE = max_esdf.y() - min_esdf.y();
  int update_XY_SIZE = (update_X_SIZE + 1) * (update_Y_SIZE + 1);
  std::vector<double> tmp_buffer1_(update_XY_SIZE,0.0), distance_buffer_(update_XY_SIZE,0.0), distance_buffer_neg_(update_XY_SIZE,0.0);
  for(int x=0;x<=update_X_SIZE;x++){
    fillESDF([&](int y){int s=gridmap_[Index2Vectornum(x+min_esdf.x(), y+min_esdf.y())];bool src=(s==Occupied)||(unknown_as_obstacle_&&s==Unknown);return src?0.0:std::numeric_limits<double>::max();},
             [&](int y,double v){tmp_buffer1_[x*update_Y_SIZE+y]=v;},0,update_Y_SIZE,update_Y_SIZE+1);
  }
  for(int y=0;y<=update_Y_SIZE;y++){
    fillESDF([&](int x){return tmp_buffer1_[x*update_Y_SIZE+y];},[&](int x,double v){distance_buffer_[x*update_Y_SIZE+y]=grid_interval_*std::sqrt(v);},0,update_X_SIZE,update_X_SIZE+1);
  }
  for(int x=0;x<=update_X_SIZE;x++){
    fillESDF([&](int y){int s=gridmap_[Index2Vectornum(x+min_esdf.x(), y+min_esdf.y())];bool free=(s==Unoccupied)||(!unknown_as_obstacle_&&s==Unknown);return free?0.0:std::numeric_limits<double>::max();},
             [&](int y,double v){tmp_buffer1_[x*update_Y_SIZE+y]=v;},0,update_Y_SIZE,update_Y_SIZE+1);
  }
  for(int y=0;y<=update_Y_SIZE;y++){
    fillESDF([&](int x){return tmp_buffer1_[x*update_Y_SIZE+y];},[&](int x,double v){distance_buffer_neg_[x*update_Y_SIZE+y]=grid_interval_*std::sqrt(v);},0,update_X_SIZE,update_X_SIZE+1);
  }
  for(int x=0;x<update_X_SIZE;x++) for(int y=0;y<update_Y_SIZE;y++){
    int g = Index2Vectornum(x+min_esdf.x(), y+min_esdf.y()); int id = x*update_Y_SIZE+y;
    distance_buffer_all_[g] = distance_buffer_[id];
    if(distance_buffer_neg_[id] > 0.0) distance_buffer_all_[g] += (-distance_buffer_neg_[id] + grid_interval_);
  }
}

template <typename F_get_val, typename F_set_val>
void SDFmap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size) {
  int v[dim_size];
  double z[dim_size + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}


void SDFmap::publish_ESDF(){
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::PointCloud2 surf_vis;
  const double min_dist = 0.0;
  const double max_dist = 5.0;
  int size = distance_buffer_all_.size();
  for(int i = 1; i< size;i++){
    Eigen::Vector2d coord = gridIndex2coordd(vectornum2gridIndex(i));
    pcl::PointXYZI pt;
    pt.x = coord.x();pt.y = coord.y();pt.z = 0.0;
    pt.intensity = std::max(min_dist, std::min(distance_buffer_all_[i],max_dist));
    cloud_vis.points.push_back(pt);
  }
  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, surf_vis);
  surf_vis.header.frame_id = "world";
  pub_ESDF_.publish(surf_vis);
}

inline double SDFmap::getDistance(const Eigen::Vector2i& id){
  // if(distance_buffer_all_[Index2Vectornum(id.x(), id.y())]>5){
  //   ROS_ERROR("out of map!!  %d  %d   distance:%f",id.x(),id.y(),distance_buffer_all_[Index2Vectornum(id[0],id[1])]);
  // }
  return distance_buffer_all_[Index2Vectornum(id[0],id[1])];
}

inline double SDFmap::getDistance(const int& idx, const int& idy){
  // if(distance_buffer_all_[Index2Vectornum(idx, idy)]>5){
  //   ROS_ERROR("out of map!!  %d  %d  distance:%f",idx,idy,distance_buffer_all_[Index2Vectornum(idx, idy)]);
  // }
  return distance_buffer_all_[Index2Vectornum(idx, idy)];
}

inline Eigen::Vector2i SDFmap::ESDFcoord2gridIndex(const Eigen::Vector2d &pt){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((pt(0) - global_x_lower_) * inv_grid_interval_ - 0.5), 0), GLX_SIZE_ - 1),
         std::min(std::max(int((pt(1) - global_y_lower_) * inv_grid_interval_ - 0.5), 0), GLY_SIZE_ - 1);
  return idx;
}

double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
    return 100;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
    return 100;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double& mindis){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
    return 1e10;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
    return 1e10;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  if(dist > mindis){
    return dist;
  }

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    return 1e10;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    return 1e10;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  return dist; 
}

double SDFmap::getDistanceReal(const Eigen::Vector2d& pos){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_) return 10000;
  return distance_buffer_all_[Index2Vectornum(coord2gridIndex(pos))];
}

double SDFmap::getUnkonwnGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
    return 100;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = coord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
    return 100;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;

  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = gridmap_[Index2Vectornum(current_idx)]==Unknown?1:0;
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

Eigen::Vector2d SDFmap::get_update_odom(){
  return update_odom_;
}

inline double SDFmap::normalize_angle(double angle){
  if(angle>M_PI) angle -= 2*M_PI;
  if(angle<-M_PI) angle += 2*M_PI;
  return angle;
}

bool SDFmap::isOccupied(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Occupied;
}

bool SDFmap::isOccupied(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Occupied;
}

bool SDFmap::isUnOccupied(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Unoccupied;
}

bool SDFmap::isUnOccupied(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Unoccupied;
}

bool SDFmap::isUnknown(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Unknown;
}

bool SDFmap::isUnknown(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Unknown;
}

bool SDFmap::isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis){
  return distance_buffer_all_[Index2Vectornum(index)] < safe_dis;
}

bool SDFmap::isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis){
  return distance_buffer_all_[Index2Vectornum(idx, idy)] < safe_dis;
}

void SDFmap::publish_gridmap(){
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;
  cloud_vis.points.reserve(GLXY_SIZE_ / 2);
  for(int idx = 0; idx < GLXY_SIZE_; ++idx){
    uint8_t occ = gridmap_[idx];
    if(occ == Occupied || occ == Unknown){
      Eigen::Vector2i lg = vectornum2gridIndex(idx);
      Eigen::Vector2d corrd = gridIndex2coordd(lg);
      pcl::PointXYZI pt;
      pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1f;
      pt.intensity = (occ == Occupied) ? 0.0f : 8.0f;
      cloud_vis.points.push_back(pt);
    }
  }
  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, map_vis);
  map_vis.header.frame_id = "world";
  map_vis.header.stamp = ros::Time::now();
  pub_gridmap_.publish(map_vis);
}

uint8_t SDFmap::CheckCollisionBycoord(const Eigen::Vector2d &pt){
  if(pt.x()>global_x_upper_||pt.x()<global_x_lower_||pt.y()>global_y_upper_||pt.y()<global_y_lower_){
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(pt);
  return gridmap_[Index2Vectornum(index)];
}

uint8_t SDFmap::CheckCollisionBycoord(const double ptx,const double pty){
  if(ptx>global_x_upper_||ptx<global_x_lower_||pty>global_y_upper_||pty<global_y_lower_){
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(Eigen::Vector2d(ptx,pty));
  return gridmap_[Index2Vectornum(index)];
}

bool SDFmap::isInGloMap(const Eigen::Vector2d &pt){
  return pt.x() < global_x_upper_ && pt.x() > global_x_lower_ && pt.y() < global_y_upper_ && pt.y() > global_y_lower_;
}

Eigen::Vector2d SDFmap::closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos){
  // move the point towards the map boundary along the ray from pos to pt
  Eigen::Vector2d diff = pt - pos;
  Eigen::Vector2d max_tc(global_x_upper_, global_y_upper_);
  Eigen::Vector2d min_tc(global_x_lower_, global_y_lower_);
  max_tc -= pos;
  min_tc -= pos;

  double min_t = 1e9;
  for (int i = 0; i < 2; ++i) {
    if (std::fabs(diff[i]) > 1e-9) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0.0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0.0 && t2 < min_t) min_t = t2;
    }
  }
  if(!std::isfinite(min_t)) {
    return pos; // degenerate, return origin
  }
  return pos + (min_t - 1e-3) * diff;
}
