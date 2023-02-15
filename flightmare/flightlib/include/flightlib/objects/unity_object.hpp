#pragma once

#include <string>

#include "flightlib/common/csv_reader.hpp"
#include "flightlib/common/rigid_state.hpp"
#include "flightlib/common/types.hpp"
namespace flightlib {
class UnityObject {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  UnityObject(std::string id, std::string prefab_id);
  virtual ~UnityObject(){};

  void run(const Scalar dt);
  // publich get functions
  virtual Vector<3> getPos(void) { return state_.p; };
  virtual Quaternion getQuat(void) { return state_.q(); };
  virtual Vector<3> getSize(void) { return size_; };
  virtual Vector<3> getScale(void) { return scale_; };
  virtual Vector<6> getBoundingBox(void) {return bounding_box_; };

  // public get functions
  const std::string getID(void) { return id_; };
  const std::string getPrefabID(void) { return prefab_id_; };
  bool isStatic(void);

  bool loadTrajectory(const std::string csv_file);

  // publich set functions
  inline void setPosition(const Vector<3>& position) { state_.p = position; };
  inline void setRotation(const Quaternion& quaternion) {
    state_.q(quaternion);
  };
  inline void setSize(const Vector<3>& size) { size_ = size; };
  inline void setScale(const Vector<3>& scale) { scale_ = scale; };
  inline void setBoundingBox(const Vector<6>& bounding_box) { bounding_box_ = bounding_box; };

 protected:
  const std::string id_;
  const std::string prefab_id_;

  Vector<6> bounding_box_; // minx miny minz maxx maxy maxz

  Scalar sign_;

  RigidState state_;

  std::vector<RigidState> traj_;

  Vector<3> size_;
  Vector<3> scale_;
};

}  // namespace flightlib