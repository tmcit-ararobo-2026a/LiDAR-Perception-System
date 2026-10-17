// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#include "lidar_perception_system/opening_tracker.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace lidar_perception_system::opening {
namespace {
constexpr double pi = 3.14159265358979323846;
struct Key {
  int x, y, z;
  bool operator==(const Key & b) const {return x == b.x && y == b.y && z == b.z;}
};
struct Hash {
  size_t operator()(const Key & k) const {
    size_t h = std::hash<int>{}(k.x);
    h ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
Key key(const Point & p, double size) {
  return {static_cast<int>(std::floor(p.x()/size)),
    static_cast<int>(std::floor(p.y()/size)), static_cast<int>(std::floor(p.z()/size))};
}
Points voxelize(const Points & input, double size, size_t limit) {
  struct Cell {Point sum{0,0,0}; int count{0};};
  std::unordered_map<Key, Cell, Hash> cells;
  for (const auto & p : input) {auto & c = cells[key(p,size)]; c.sum += p; ++c.count;}
  std::vector<std::pair<Key, Point>> ordered;
  ordered.reserve(cells.size());
  for (const auto & item : cells) {ordered.push_back({item.first,item.second.sum/item.second.count});}
  // Stable spatial ordering also makes decimation independent of hash iteration order.
  std::sort(ordered.begin(),ordered.end(),[](const auto & a,const auto & b) {
    return std::tie(a.first.x,a.first.y,a.first.z)<std::tie(b.first.x,b.first.y,b.first.z);
  });
  Points output;
  const size_t stride = std::max(size_t{1}, (ordered.size()+limit-1)/limit);
  for (size_t i=0;i<ordered.size();i+=stride) {output.push_back(ordered[i].second);}
  return output;
}
double angle(double a) {return std::atan2(std::sin(a),std::cos(a));}
double arc_span(std::vector<double> a) {
  if (a.size()<2) {return 0;}
  std::sort(a.begin(),a.end());
  double gap=a.front()+2*pi-a.back();
  for (size_t i=1;i<a.size();++i) {gap=std::max(gap,a[i]-a[i-1]);}
  return 2*pi-gap;
}
double median(std::vector<double> a) {
  auto middle=a.begin()+a.size()/2;
  std::nth_element(a.begin(),middle,a.end());
  return *middle;
}
Points local_points(const Points & p,const Pose & pose) {
  Points output; output.reserve(p.size());
  for (const auto & v:p) {output.push_back(pose.local(v));}
  return output;
}
}  // namespace

Point Pose::world(const Point & p) const {
  const double c=std::cos(yaw),s=std::sin(yaw);
  return {c*p.x()-s*p.y()+xy.x(),s*p.x()+c*p.y()+xy.y(),p.z()};
}
Point Pose::local(const Point & p) const {
  const double c=std::cos(yaw),s=std::sin(yaw),x=p.x()-xy.x(),y=p.y()-xy.y();
  return {c*x+s*y,-s*x+c*y,p.z()};
}
void Config::validate() const {
  for (double v : {min_x,max_x,min_y,max_y,min_z,max_z,self_radius,body_min_z,body_max_z,
      cluster_resolution,min_span,max_span,crop_margin,body_voxel,opening_voxel,window_s,
      correspondence_m,max_registration_rmse,min_overlap,min_opening_z,max_opening_z,
      min_radius,max_radius,nominal_radius,slice_m,max_residual,max_plane_rmse,min_arc,
      height_change_m,min_lip_width,prediction_s,reset_s,association_m}) {
    if (!std::isfinite(v)) {throw std::invalid_argument("Opening config must be finite");}
  }
  if (min_x>=max_x || min_y>=max_y || min_z>=max_z ||
    std::max({std::abs(min_x),std::abs(max_x),std::abs(min_y),std::abs(max_y),std::abs(max_z)})>100 ||
    body_min_z<min_z || body_min_z>=body_max_z || body_max_z>=min_opening_z ||
    min_opening_z>=max_opening_z || max_opening_z>max_z || self_radius<0 || crop_margin<0 ||
    min_span<=0 || max_span<min_span || cluster_resolution<0.01 || body_voxel<0.005 ||
    opening_voxel<0.001 || min_radius<=0 || min_radius>=max_radius || nominal_radius<min_radius ||
    nominal_radius>max_radius || slice_m<=0 || max_residual<=0 || max_plane_rmse<=0 ||
    min_arc<=0 || min_arc>2*pi || height_change_m<=0 || min_lip_width<0 || min_lip_width>max_residual || window_s<=0 || window_s>5 ||
    prediction_s<=0 || prediction_s>reset_s || reset_s>10 || correspondence_m<=0 ||
    correspondence_m>1 || max_registration_rmse<=0 || min_overlap<=0 || min_overlap>1 ||
    association_m<=0 || icp_iterations<1 || icp_iterations>100 || ransac_iterations<1 ||
    ransac_iterations>1000 || min_body_points<3 || min_rim_points<3 || min_current_points<3 ||
    min_side_points<1 || confirm_hits<1 || max_body_points<min_body_points ||
    max_body_points>20000 || max_model_points<max_body_points || max_model_points>200000 ||
    max_frame_points<max_model_points || max_frame_points>1000000) {
    throw std::invalid_argument("Invalid opening tracker bounds or limits");
  }
}
Tracker::Tracker(Config config):cfg_(config) {cfg_.validate();}
void Tracker::reset() {
  frames_.clear(); selected_.clear(); accumulated_.clear(); reference_body_.clear();
  have_body_=false; confirmed_=false; height_pending_=false; hits_=0;
  last_opening_.reset(); body_stamp_=0; input_stamp_=-1; observation_stamp_=0;
  body_velocity_.setZero(); velocity_.setZero(); yaw_rate_=0;
}
void Tracker::set_exclusions(const std::vector<Eigen::Vector3d> & masks) {exclusions_=masks;}

std::vector<Tracker::Cluster> Tracker::clusters(const Points & points) const {
  std::unordered_map<Key,Points,Hash> cells;
  for (const auto & p:points) {
    if (p.z()<cfg_.body_min_z || p.z()>cfg_.body_max_z) {continue;}
    auto k=key(p,cfg_.cluster_resolution); k.z=0; cells[k].push_back(p);
  }
  std::vector<Key> starts;
  for (const auto & c:cells) {starts.push_back(c.first);}
  std::sort(starts.begin(),starts.end(),[](const auto & a,const auto & b) {
    return std::tie(a.x,a.y)<std::tie(b.x,b.y);
  });
  std::unordered_set<Key,Hash> visited;
  std::vector<Cluster> result;
  for (const auto & start:starts) {
    if (!visited.insert(start).second) {continue;}
    std::vector<Key> todo{start};
    Cluster c; c.center.setZero();
    c.min_x=c.min_y=std::numeric_limits<double>::infinity();
    c.max_x=c.max_y=-c.min_x;
    for (size_t i=0;i<todo.size();++i) {
      for (const auto & p:cells.at(todo[i])) {
        c.body.push_back(p); c.center+=p.head<2>();
        c.min_x=std::min(c.min_x,p.x()); c.max_x=std::max(c.max_x,p.x());
        c.min_y=std::min(c.min_y,p.y()); c.max_y=std::max(c.max_y,p.y());
      }
      for (int dx=-1;dx<=1;++dx) {for (int dy=-1;dy<=1;++dy) {
        const Key n{todo[i].x+dx,todo[i].y+dy,0};
        if (cells.count(n) && visited.insert(n).second) {todo.push_back(n);}
      }}
    }
    if (static_cast<int>(c.body.size())<cfg_.min_body_points ||
      c.max_x-c.min_x<cfg_.min_span || c.max_y-c.min_y<cfg_.min_span ||
      c.max_x-c.min_x>cfg_.max_span || c.max_y-c.min_y>cfg_.max_span) {continue;}
    c.center/=c.body.size();
    c.body=voxelize(c.body,cfg_.body_voxel,cfg_.max_body_points);
    result.push_back(std::move(c));
  }
  return result;
}

bool Tracker::register_body(const Points & body,const Pose & initial,Pose & output,
  double & rmse,double & overlap) const {
  if (reference_body_.empty()) {return false;}
  std::unordered_map<Key,std::vector<size_t>,Hash> hash;
  for (size_t i=0;i<reference_body_.size();++i) {
    hash[key(reference_body_[i],cfg_.correspondence_m)].push_back(i);
  }
  Pose estimate=initial;
  struct Pair {Point p,q; double squared;};
  for (int iteration=0;iteration<=cfg_.icp_iterations;++iteration) {
    std::vector<Pair> pairs;
    for (const auto & map:body) {
      const Point p=estimate.local(map); const auto k=key(p,cfg_.correspondence_m);
      double best=cfg_.correspondence_m*cfg_.correspondence_m; const Point * q=nullptr;
      for (int dx=-1;dx<=1;++dx) {for (int dy=-1;dy<=1;++dy) {for (int dz=-1;dz<=1;++dz) {
        auto cell=hash.find({k.x+dx,k.y+dy,k.z+dz}); if (cell==hash.end()) {continue;}
        for (size_t i:cell->second) {
          double d=(p-reference_body_[i]).squaredNorm();
          if (d<best) {best=d;q=&reference_body_[i];}
        }
      }}}
      if (q) {pairs.push_back({p,*q,best});}
    }
    overlap=static_cast<double>(pairs.size())/std::max(size_t{1},body.size());
    if (overlap<cfg_.min_overlap || pairs.size()<12) {return false;}
    std::sort(pairs.begin(),pairs.end(),[](const auto & a,const auto & b) {return a.squared<b.squared;});
    pairs.resize(std::max(size_t{12},pairs.size()*7/10));
    Eigen::Matrix3d h=Eigen::Matrix3d::Zero(); Point g=Point::Zero(); double sum=0;
    for (const auto & pair:pairs) {
      const Eigen::Vector2d e=(pair.p-pair.q).head<2>();
      Eigen::Matrix<double,2,3> j; j<<1,0,-pair.p.y(),0,1,pair.p.x();
      const double w=std::min(1.0,0.03/std::max(1e-9,std::sqrt(pair.squared)));
      h.noalias()+=w*j.transpose()*j; g.noalias()+=w*j.transpose()*e; sum+=pair.squared;
    }
    rmse=std::sqrt(sum/pairs.size());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(h);
    if (eigen.info()!=Eigen::Success || eigen.eigenvalues().minCoeff()<1e-5 ||
      eigen.eigenvalues().minCoeff()/eigen.eigenvalues().maxCoeff()<1e-4) {return false;}
    if (iteration==cfg_.icp_iterations) {break;}
    Point step=-h.ldlt().solve(g);
    if (!step.allFinite()) {return false;}
    const double scaling=std::max({1.0,step.head<2>().norm()/0.1,std::abs(step.z())/0.15});
    step/=scaling;
    // Apply the local alignment increment; the inverse is the map pose update.
    Pose incremental; incremental.xy=step.head<2>(); incremental.yaw=step.z();
    const Point new_origin=estimate.world(incremental.local(Point::Zero()));
    estimate.xy=new_origin.head<2>(); estimate.yaw=angle(estimate.yaw-step.z());
    if (step.norm()<1e-5) {break;}
  }
  if (rmse>cfg_.max_registration_rmse || (estimate.xy-initial.xy).norm()>cfg_.association_m ||
    std::abs(angle(estimate.yaw-initial.yaw))>0.65) {return false;}
  output=estimate; return true;
}

std::optional<Opening> Tracker::fit_opening(const Points & input,const Points & current) const {
  const auto points=voxelize(input,cfg_.opening_voxel,cfg_.max_model_points);
  std::optional<Opening> best;
  std::mt19937 rng(0x20261018);
  for (double z=cfg_.min_opening_z;z<=cfg_.max_opening_z+1e-6;z+=cfg_.slice_m/2) {
    std::vector<size_t> slice;
    for (size_t i=0;i<points.size();++i) {
      if (std::abs(points[i].z()-z)<=cfg_.slice_m/2) {slice.push_back(i);}
    }
    if (static_cast<int>(slice.size())<cfg_.min_rim_points) {continue;}
    for (int iteration=0;iteration<cfg_.ransac_iterations;++iteration) {
      const Point & a=points[slice[rng()%slice.size()]],& b=points[slice[rng()%slice.size()]],
        & c=points[slice[rng()%slice.size()]];
      Eigen::Matrix2d m; m.row(0)=2*(b-a).head<2>(); m.row(1)=2*(c-a).head<2>();
      if (std::abs(m.determinant())<1e-5) {continue;}
      Eigen::Vector2d rhs(b.head<2>().squaredNorm()-a.head<2>().squaredNorm(),
        c.head<2>().squaredNorm()-a.head<2>().squaredNorm());
      Eigen::Vector2d center=m.inverse()*rhs;
      double radius=(center-a.head<2>()).norm();
      if (radius<cfg_.min_radius || radius>cfg_.max_radius) {continue;}
      std::vector<size_t> inliers;
      for (size_t i:slice) {
        if (std::abs((points[i].head<2>()-center).norm()-radius)<=cfg_.max_residual) {
          inliers.push_back(i);
        }
      }
      if (static_cast<int>(inliers.size())<cfg_.min_rim_points) {continue;}
      for (int refine=0;refine<5;++refine) {
        Eigen::Matrix3d h=Eigen::Matrix3d::Zero(); Point g=Point::Zero();
        for (size_t i:inliers) {
          const Eigen::Vector2d d=center-points[i].head<2>(); const double norm=d.norm();
          if (norm<1e-6) {continue;}
          Point j(d.x()/norm,d.y()/norm,-1); const double e=norm-radius;
          h.noalias()+=j*j.transpose(); g+=j*e;
        }
        if (std::abs(h.determinant())<1e-6) {break;}
        Point step=h.ldlt().solve(g); if (!step.allFinite()) {break;}
        center-=step.head<2>(); radius-=step.z();
        if (step.norm()<1e-5) {break;}
      }
      if (radius<cfg_.min_radius || radius>cfg_.max_radius) {continue;}
      std::vector<double> heights,angles,radii; double sum=0;
      for (size_t i:inliers) {heights.push_back(points[i].z());}
      const double top=median(heights); double plane=0;
      if (top<cfg_.min_opening_z || top>cfg_.max_opening_z) {continue;}
      for (size_t i:inliers) {
        const auto d=points[i].head<2>()-center;
        const double e=d.norm()-radius; sum+=e*e;
        radii.push_back(d.norm());
        plane+=std::pow(points[i].z()-top,2); angles.push_back(std::atan2(d.y(),d.x()));
      }
      const double residual=std::sqrt(sum/inliers.size()),span=arc_span(angles);
      const double plane_rmse=std::sqrt(plane/inliers.size());
      if (residual>cfg_.max_residual || span<cfg_.min_arc || plane_rmse>cfg_.max_plane_rmse) {continue;}
      std::sort(radii.begin(),radii.end());
      // A horizontal lip has radial extent. An isolated wall contour alone is
      // not enough evidence that its highest visible return is the true rim.
      if (radii[radii.size()*9/10]-radii[radii.size()/10]<cfg_.min_lip_width) {continue;}
      int below=0,above=0;
      for (const auto & p:points) {
        const double radial=(p.head<2>()-center).norm();
        if (p.z()<top-0.03 && p.z()>top-0.22 && std::abs(radial-radius)<0.06) {++below;}
        if (p.z()>top+cfg_.slice_m/2+0.002 && p.z()<top+0.15 && std::abs(radial-radius)<0.025) {++above;}
      }
      // A side-wall cross section has continuing shell above it, unlike a top edge.
      if (below<cfg_.min_side_points || above>=3) {continue;}
      int fresh=0; std::vector<double> fresh_angles;
      for (const auto & p:current) {
        const auto d=p.head<2>()-center;
        if (std::abs(p.z()-top)<=cfg_.slice_m/2 && std::abs(d.norm()-radius)<=cfg_.max_residual) {
          ++fresh; fresh_angles.push_back(std::atan2(d.y(),d.x()));
        }
      }
      if (fresh<cfg_.min_current_points || arc_span(fresh_angles)<0.35) {continue;}
      Opening candidate; candidate.center={center.x(),center.y(),top}; candidate.radius=radius;
      candidate.residual=residual; candidate.plane_rmse=plane_rmse; candidate.arc=span;
      candidate.points=static_cast<int>(inliers.size()); candidate.current_points=fresh;
      candidate.score=2*span+std::log1p(candidate.points)-100*residual-100*plane_rmse;
      const bool same_axis=best && (candidate.center.head<2>()-best->center.head<2>()).norm()<0.04;
      if (!best || (same_axis && candidate.center.z()>best->center.z()+0.008) ||
        (!(same_axis && best->center.z()>candidate.center.z()+0.008) && candidate.score>best->score)) {best=candidate;}
    }
  }
  return best;
}

void Tracker::rebuild(double stamp) {
  while (!frames_.empty() && stamp-frames_.front().stamp>cfg_.window_s) {frames_.pop_front();}
  Points body,upper;
  for (const auto & frame:frames_) {
    body.insert(body.end(),frame.body.begin(),frame.body.end());
    upper.insert(upper.end(),frame.upper.begin(),frame.upper.end());
  }
  // Keep a last-good registration reference during short occlusions. Its age never
  // counts as a new opening observation, and it is removed by the reset deadline.
  if (!body.empty()) {reference_body_=voxelize(body,cfg_.body_voxel,cfg_.max_body_points);}
  accumulated_=voxelize(upper,cfg_.opening_voxel,cfg_.max_model_points);
}
Result Tracker::missing(double stamp,const std::string & reason,Result result) {
  result.stamp=stamp;result.track_id=track_id_; result.pose=pose_; result.reason=reason;
  result.observation_stamp=observation_stamp_; result.model_points=static_cast<int>(accumulated_.size());
  if (confirmed_ && !height_pending_ && stamp>=observation_stamp_ && stamp-observation_stamp_<cfg_.prediction_s) {
    result.valid=true; result.predicted=true; result.position=position_+velocity_*(stamp-observation_stamp_);
    result.position.z()=position_.z(); result.velocity=velocity_; result.opening=last_opening_;
    result.position_variance=variance_+0.1*(stamp-observation_stamp_);
  }
  return result;
}
Result Tracker::process(const Points & map_points,double stamp,const Eigen::Vector2d & sensor_xy) {
  Result result;
  selected_.clear();
  if (!std::isfinite(stamp) || stamp<=0 || !sensor_xy.allFinite()) {return missing(stamp,"invalid input time/pose");}
  if (input_stamp_>=0 && stamp<input_stamp_) {reset();}
  if (stamp==input_stamp_) {result.reason="duplicate input";result.stamp=stamp;return result;}
  input_stamp_=stamp;
  if (have_body_ && stamp-body_stamp_>cfg_.reset_s) {reset();input_stamp_=stamp;}
  rebuild(stamp);
  Points filtered;filtered.reserve(std::min(map_points.size(),static_cast<size_t>(cfg_.max_frame_points)));
  // Uniform input decimation bounds work without retaining the first spatial sector only.
  const size_t stride=std::max(size_t{1},(map_points.size()+cfg_.max_frame_points-1)/cfg_.max_frame_points);
  for (size_t i=0;i<map_points.size();i+=stride) {
    const auto & p=map_points[i];
    if (!p.allFinite() || p.x()<cfg_.min_x || p.x()>cfg_.max_x || p.y()<cfg_.min_y ||
      p.y()>cfg_.max_y || p.z()<cfg_.min_z || p.z()>cfg_.max_z ||
      (p.head<2>()-sensor_xy).norm()<cfg_.self_radius) {continue;}
    filtered.push_back(p);
  }
  auto candidates=clusters(filtered);result.candidates=static_cast<int>(candidates.size());
  const double elapsed=have_body_?stamp-body_stamp_:0;
  const Eigen::Vector2d hint=cluster_center_+body_velocity_*elapsed;
  Cluster * chosen=nullptr; double best=-1e9,second=-1e9;
  for (auto & c:candidates) {
    if (have_body_ && (c.center-hint).norm()>cfg_.association_m+0.5*elapsed) {continue;}
    bool excluded=false;
    if (!have_body_) {
      for (const auto & mask:exclusions_) {
        if ((c.center-mask.head<2>()).norm()<mask.z()) {excluded=true;break;}
      }
    }
    if (excluded) {continue;}
    int upper=0;
    for (const auto & p:filtered) {
      if (p.z()>=cfg_.min_opening_z && p.x()>c.min_x-cfg_.crop_margin &&
        p.x()<c.max_x+cfg_.crop_margin && p.y()>c.min_y-cfg_.crop_margin &&
        p.y()<c.max_y+cfg_.crop_margin) {++upper;}
    }
    double score=have_body_?-(c.center-hint).norm():std::log1p(c.body.size())+std::log1p(upper);
    if (score>best) {second=best;best=score;chosen=&c;} else {second=std::max(second,score);}
  }
  if (!chosen) {if (!confirmed_) {hits_=0;} return missing(stamp,"no robot cluster",result);}
  if (!have_body_ && second>best-0.5) {return missing(stamp,"ambiguous robot clusters",result);}
  Pose updated;
  if (!have_body_) {
    pose_.xy=chosen->center;pose_.yaw=0;updated=pose_;have_body_=true;track_id_=++next_id_;
    result.overlap=1;
  } else {
    Pose initial=pose_; initial.xy+=body_velocity_*elapsed; initial.yaw=angle(initial.yaw+yaw_rate_*elapsed);
    if (!register_body(chosen->body,initial,updated,result.registration_rmse,result.overlap)) {
      if (!confirmed_) {hits_=0;} return missing(stamp,"robot registration rejected",result);
    }
    if (elapsed>0) {
      const auto v=(updated.xy-pose_.xy)/elapsed;
      body_velocity_=0.5*body_velocity_+0.5*v;
      yaw_rate_=0.5*yaw_rate_+0.5*angle(updated.yaw-pose_.yaw)/elapsed;
    }
  }
  pose_=updated;body_stamp_=stamp;cluster_center_=chosen->center;result.body_valid=true;
  for (const auto & p:filtered) {
    if (p.x()>=chosen->min_x-cfg_.crop_margin && p.x()<=chosen->max_x+cfg_.crop_margin &&
      p.y()>=chosen->min_y-cfg_.crop_margin && p.y()<=chosen->max_y+cfg_.crop_margin) {selected_.push_back(p);}
  }
  Points upper;
  for (const auto & p:selected_) {if (p.z()>cfg_.min_opening_z-0.25) {upper.push_back(pose_.local(p));}}
  upper=voxelize(upper,cfg_.opening_voxel,cfg_.max_model_points);
  auto instantaneous=fit_opening(upper,upper);
  if (instantaneous && last_opening_ &&
    std::abs(instantaneous->center.z()-last_opening_->center.z())>cfg_.height_change_m) {
    for (auto & frame:frames_) {frame.upper.clear();}
    confirmed_=false;hits_=0;height_pending_=true;last_opening_.reset();++height_resets_;
  }
  Frame frame; frame.stamp=stamp;frame.body=local_points(chosen->body,pose_);frame.upper=upper;
  frames_.push_back(std::move(frame));
  // At most 100 accepted scans per second; protect memory under pathological timestamp rates.
  while (frames_.size()>100) {frames_.pop_front();}
  rebuild(stamp);
  auto opening=fit_opening(accumulated_,upper);
  if (opening && last_opening_ &&
    std::abs(opening->center.z()-last_opening_->center.z())>cfg_.height_change_m) {
    for (auto & previous:frames_) {previous.upper.clear();}
    frames_.back().upper=upper;rebuild(stamp);opening=fit_opening(accumulated_,upper);
    confirmed_=false;hits_=0;height_pending_=true;last_opening_.reset();++height_resets_;
  }
  if (!opening) {if (!confirmed_) {hits_=0;} return missing(stamp,"no fresh opening edge",result);}
  if (last_opening_ && (opening->center.head<2>()-last_opening_->center.head<2>()).norm()>0.15) {
    return missing(stamp,"opening association rejected",result);
  }
  const Point measurement=pose_.world(opening->center);
  const double dt=stamp-observation_stamp_;
  if (confirmed_ && dt>0 && dt<cfg_.reset_s) {
    const Point v=(measurement-measured_)/dt;
    if (v.head<2>().norm()>5.0) {return missing(stamp,"opening velocity rejected",result);}
    velocity_=0.5*velocity_+0.5*v;velocity_.z()=0;
    position_=0.6*measurement+0.4*(position_+velocity_*dt);position_.z()=measurement.z();
  } else {position_=measurement;velocity_.setZero();}
  measured_=measurement;observation_stamp_=stamp;last_opening_=opening;
  hits_=std::min(cfg_.confirm_hits,hits_+1);confirmed_=hits_>=cfg_.confirm_hits;
  if (confirmed_) {height_pending_=false;}
  variance_=std::max(0.0004,std::pow(opening->residual,2)+std::pow(result.registration_rmse,2));
  result=missing(stamp,confirmed_?"opening observed":"confirming opening",result);
  result.valid=confirmed_;result.predicted=false;result.position=position_;result.velocity=velocity_;
  result.opening=opening;result.position_variance=variance_;
  result.confidence=std::clamp(opening->arc/(2*pi)*(1-opening->residual/cfg_.max_residual),0.0,1.0);
  return result;
}
}  // namespace lidar_perception_system::opening
