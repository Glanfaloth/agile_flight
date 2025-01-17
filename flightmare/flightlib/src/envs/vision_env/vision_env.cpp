// This code is used for reinforcement learning, which does not use any ROS.

// when you change the vision_env.cpp, you should do

// cd flightlib/build
// cmake ..
// make -j10
// pip install . 

// by doing so, the flightgym python package will be updated.
// the flightgym package is used in the run_vision_ppo.py

#include "flightlib/envs/vision_env/vision_env.hpp"

namespace flightlib {

VisionEnv::VisionEnv()
  : VisionEnv(getenv("FLIGHTMARE_PATH") +
                std::string("/flightpy/configs/vision/config.yaml"),
              0) {}

VisionEnv::VisionEnv(const std::string &cfg_path, const int env_id)
  : EnvBase() {
  // check if configuration file exist
  if (!(file_exists(cfg_path))) {
    logger_.error("Configuration file %s does not exists.", cfg_path);
  }
  // load configuration file
  cfg_ = YAML::LoadFile(cfg_path);
  //
  init();
  env_id_ = env_id;
}

VisionEnv::VisionEnv(const YAML::Node &cfg_node, const int env_id) : EnvBase() {
  cfg_ = cfg_node;

  //
  init();
  env_id_ = env_id;
}

void VisionEnv::init() {
  //
  unity_render_offset_ << 0.0, 0.0, 0.0;
  // goal_linear_vel_ << 0.0, 0.0, 0.0;
  cmd_.setZeros();
  index = 0;

  // create quadrotors
  quad_ptr_ = std::make_shared<Quadrotor>();

  // // set the quadrotor size
  // Vector<3> quad_size = Vector<3>(0.1, 0.1, 0.1);
  // quad_ptr_->setSize(quad_size);

  // update dynamics
  QuadrotorDynamics dynamics;
  dynamics.updateParams(cfg_);
  quad_ptr_->updateDynamics(dynamics);

  // define input and output dimension for the environment
  obs_dim_ = visionenv::kNObs;
  act_dim_ = visionenv::kNAct;
  rew_dim_ = 0;
  num_detected_obstacles_ = visionenv::kNObstacles;

  // load parameters
  loadParam(cfg_);

  // add camera
  if (!configCamera(cfg_)) {
    logger_.error(
      "Cannot config RGB Camera. Something wrong with the config file");
  }

  obstacle_cfg_path_ = getenv("FLIGHTMARE_PATH") +
                       std::string("/flightpy/configs/vision/") +
                       difficulty_level_ + std::string("/") + env_folder_;

  // add dynamic objects
  std::string dynamic_object_yaml =
    obstacle_cfg_path_ + std::string("/dynamic_obstacles.yaml");

  if (!configDynamicObjects(dynamic_object_yaml)) {
    logger_.error(
      "Cannot config Dynamic Object Yaml. Something wrong with the config "
      "file");
  }

  // add static objects
  static_object_csv_ =
    obstacle_cfg_path_ + std::string("/static_obstacles.csv");
  if (!configStaticObjects(static_object_csv_)) {
    logger_.error(
      "Cannot config Static Object. Something wrong with the config file");
  }

  // use single rotor control or bodyrate control
  Scalar max_force = quad_ptr_->getDynamics().getForceMax();
  Vector<3> max_omega = quad_ptr_->getDynamics().getOmegaMax();
  //
  act_mean_ << (max_force / quad_ptr_->getMass()) / 2, 0.0, 0.0, 0.0;
  act_std_ << (max_force / quad_ptr_->getMass()) / 2, max_omega.x(),
    max_omega.y(), max_omega.z();
}

VisionEnv::~VisionEnv() {}

bool VisionEnv::reset(Ref<Vector<>> obs) {
  quad_state_.setZero();
  pi_act_.setZero();
  old_pi_act_.setZero();
  if (!test_env_) {
    start_pos_ << uniform_dist_(random_gen_) * 3, uniform_dist_(random_gen_) * 3, uniform_dist_(random_gen_) * 0.5 + 0.5;
    goal_pos_ << uniform_dist_(random_gen_) * 3, uniform_dist_(random_gen_) * 3, uniform_dist_(random_gen_) * 0.5 + 0.5;
  }

  // reset position
  quad_state_.x(QS::POSX) = start_pos_[0];
  quad_state_.x(QS::POSY) = start_pos_[1];
  quad_state_.x(QS::POSZ) = start_pos_[2];

  // reset quadrotor with random states
  quad_ptr_->reset(quad_state_);

  // reset control command
  cmd_.t = 0.0;
  // use collective thrust and bodyrate control mode
  cmd_.setCmdMode(quadcmd::THRUSTRATE);
  cmd_.collective_thrust = 0;
  cmd_.omega.setZero();

  // obtain observations
  getObs(obs);
  return true;
}

bool VisionEnv::reset(Ref<Vector<>> obs, bool random) { return reset(obs); }

bool VisionEnv::getObs(Ref<Vector<>> obs) {
  if (obs.size() != obs_dim_) {
    logger_.error("Observation dimension mismatch. %d != %d", obs.size(),
                  obs_dim_);
    return false;
  }
  // compute rotation matrix
  Vector<9> ori = Map<Vector<>>(quad_state_.R().data(), quad_state_.R().size());

  quad_ptr_->getState(&quad_state_);

  // Observations
  obs << ori, quad_state_.v, goal_pos_ - quad_state_.p;
  return true;
}

bool VisionEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs,
                     Ref<Vector<>> reward) {
  if (!act.allFinite() || act.rows() != act_dim_ || rew_dim_ != reward.rows()) {
    return false;
    logger_.error(
      "Cannot run environment simulation. dimension mismatch or invalid "
      "actions.");
  }

  //
  old_pi_act_ = pi_act_;

  // compute actual control actions
  // act has range between [-1, 1] due to Tanh layer of the NN policy
  pi_act_ = act.cwiseProduct(act_std_) + act_mean_;

  cmd_.t += sim_dt_;
  quad_state_.t += sim_dt_;

  // apply old actions to simulate delay
  cmd_.collective_thrust = old_pi_act_(0);
  cmd_.omega = old_pi_act_.segment<3>(1);

  // simulate quadrotor
  quad_ptr_->run(cmd_, sim_dt_);

  // update quadrotor state and old quad_state
  quad_old_state_ = quad_state_;
  quad_ptr_->getState(&quad_state_);

  // simulate dynamic obstacles
  simDynamicObstacles(sim_dt_);

  // update observations
  getObs(obs);

  return computeReward(reward);
}

bool VisionEnv::simDynamicObstacles(const Scalar dt) {
  if (dynamic_objects_.size() <= 0) {
    logger_.warn(
      "No Dynamic Obstacles defined. Skipping dynamic obstacles simulation.");
    return false;
  }
  for (int i = 0; i < int(dynamic_objects_.size()); i++) {
    dynamic_objects_[i]->run(sim_dt_);
  }
  return true;
}

bool VisionEnv::computeReward(Ref<Vector<>> reward) {
  // ---------------------- reward function design
  // - position tracking
  const Scalar pos_penalty = pos_coeff_ * std::exp(-(quad_state_.p - goal_pos_).norm());

  // - orientation tracking
  const Scalar ori_penalty =
    ori_coeff_ *
    std::exp(-(quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0)).norm());

  // - linear velocity tracking
  const Scalar lin_vel_penalty = vel_coeff_ * std::exp(-quad_state_.v.norm());

  // - angular velocity tracking to avoid oscillations
  const Scalar ang_vel_penalty = angular_vel_coeff_ * std::exp(-quad_state_.w.norm());

  //  change progress reward as survive reward
  const Scalar total_reward =
    pos_penalty + ori_penalty + lin_vel_penalty + ang_vel_penalty + survive_rew_;
  
  std::cout << "quad_state_.p = " << quad_state_.p.transpose() << std::endl;
  if (test_env_ && (quad_state_.p - goal_pos_).norm() < 0.5) {
    std::cout << "update" << std::endl;
    index = index + 1;
    std::vector<Scalar> goal_pos_vec = cfg_["environment"]["way_points"][index+1].as<std::vector<Scalar>>();
    goal_pos_ = Vector<3>(goal_pos_vec.data());
  }

  // return all reward components for debug purposes
  // only the total reward is used by the RL algorithm
  reward << pos_penalty, ori_penalty, lin_vel_penalty, ang_vel_penalty, survive_rew_,
    total_reward;

  return true;
}

bool VisionEnv::isTerminalState(Scalar &reward) {
  // simulation time out
  if (cmd_.t >= max_t_ - sim_dt_) {
    reward = 0.0;
    return true;
  }

  return false;
}

bool VisionEnv::getQuadAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && pi_act_.allFinite() && (act.size() == pi_act_.size())) {
    act = pi_act_;
    return true;
  }
  return false;
}

bool VisionEnv::getQuadState(Ref<Vector<>> obs) const {
  if (quad_state_.t >= 0.0 && (obs.rows() == visionenv::kNQuadState)) {
    obs << quad_state_.t, quad_state_.p, quad_state_.qx, quad_state_.v,
      quad_state_.w, quad_state_.a, quad_ptr_->getMotorOmega(),
      quad_ptr_->getMotorThrusts();
    return true;
  }
  logger_.error("Get Quadrotor state failed.");
  return false;
}

bool VisionEnv::getDepthImage(Ref<DepthImgVector<>> depth_img) {
  if (!rgb_camera_ || !rgb_camera_->getEnabledLayers()[0]) {
    logger_.error(
      "No RGB Camera or depth map is not enabled. Cannot retrieve depth "
      "images.");
    return false;
  }
  rgb_camera_->getDepthMap(depth_img_);

  depth_img = Map<DepthImgVector<>>((float_t *)depth_img_.data,
                                    depth_img_.rows * depth_img_.cols);
  return true;
}

bool VisionEnv::getImage(Ref<ImgVector<>> img, const bool rgb) {
  if (!rgb_camera_) {
    logger_.error("No Camera! Cannot retrieve Images.");
    return false;
  }

  rgb_camera_->getRGBImage(rgb_img_);

  if (rgb_img_.rows != img_height_ || rgb_img_.cols != img_width_) {
    logger_.error(
      "Image resolution mismatch. Aborting.. Image rows %d != %d, Image cols "
      "%d != %d",
      rgb_img_.rows, img_height_, rgb_img_.cols, img_width_);
    return false;
  }

  if (!rgb) {
    // converting rgb image to gray image
    cvtColor(rgb_img_, gray_img_, CV_RGB2GRAY);
    // map cv::Mat data to Eiegn::Vector
    img = Map<ImgVector<>>(gray_img_.data, gray_img_.rows * gray_img_.cols);
  } else {
    img = Map<ImgVector<>>(rgb_img_.data, rgb_img_.rows * rgb_img_.cols *
                                            rgb_camera_->getChannels());
  }
  return true;
}

bool VisionEnv::loadParam(const YAML::Node &cfg) {
  if (cfg["environment"]) {
    difficulty_level_ = cfg["environment"]["level"].as<std::string>();
    env_folder_ = cfg["environment"]["env_folder"].as<std::string>();
    world_box_ = cfg["environment"]["world_box"].as<std::vector<Scalar>>();
    test_env_ = cfg["environment"]["test_env"].as<bool>();
    if (test_env_) {
      // std::vector<Scalar> start_pos_vec = cfg["environment"]["start_pos"].as<std::vector<Scalar>>();
      // start_pos_ = Vector<3>(start_pos_vec.data());
      // std::vector<Scalar> goal_pos_vec = cfg["environment"]["goal_pos"].as<std::vector<Scalar>>();
      // goal_pos_ = Vector<3>(goal_pos_vec.data());
      std::vector<Scalar> start_pos_vec = cfg["environment"]["way_points"][index].as<std::vector<Scalar>>();
      start_pos_ = Vector<3>(start_pos_vec.data());
      quad_old_state_.p = start_pos_;
      std::vector<Scalar> goal_pos_vec = cfg["environment"]["way_points"][index+1].as<std::vector<Scalar>>();
      goal_pos_ = Vector<3>(goal_pos_vec.data());
    } else {
      start_pos_ << uniform_dist_(random_gen_) * 3, uniform_dist_(random_gen_) * 3, uniform_dist_(random_gen_) * 0.5 + 0.5;
      goal_pos_ << uniform_dist_(random_gen_) * 3, uniform_dist_(random_gen_) * 3, uniform_dist_(random_gen_) * 0.5 + 0.5;
    }
    num_waypoints = cfg["environment"]["way_points"].size();
  }

  if (cfg["simulation"]) {
    sim_dt_ = cfg["simulation"]["sim_dt"].as<Scalar>();
    max_t_ = cfg["simulation"]["max_t"].as<Scalar>();
  } else {
    logger_.error("Cannot load [quadrotor_env] parameters");
    return false;
  }

  if (cfg["rewards"]) {
    // load reward coefficients for reinforcement learning
    vel_coeff_ = cfg["rewards"]["vel_coeff"].as<Scalar>();
    angular_vel_coeff_ = cfg["rewards"]["angular_vel_coeff"].as<Scalar>();
    survive_rew_ = cfg["rewards"]["survive_rew"].as<Scalar>();
    pos_coeff_ = cfg["rewards"]["pos_coeff"].as<Scalar>();
    ori_coeff_ = cfg["rewards"]["ori_coeff"].as<Scalar>();

    // load reward settings
    reward_names_ = cfg["rewards"]["names"].as<std::vector<std::string>>();
    rew_dim_ = reward_names_.size();
  } else {
    logger_.error("Cannot load [rewards] parameters");
    return false;
  }

  // environment
  if (cfg["unity"]) {
    unity_render_ = cfg["unity"]["render"].as<bool>();
    scene_id_ = cfg["unity"]["scene_id"].as<SceneID>();
  }

  //
  std::string scene_file =
    getenv("FLIGHTMARE_PATH") + std::string("/flightpy/configs/scene.yaml");
  // check if configuration file exist
  if (!(file_exists(scene_file))) {
    logger_.error("Unity scene configuration file %s does not exists.",
                  scene_file);
  }
  // load configuration file
  YAML::Node scene_cfg_node = YAML::LoadFile(scene_file);
  std::string scene_idx = "scene_" + std::to_string(scene_id_);

  std::vector<Scalar> render_offset =
    scene_cfg_node[scene_idx]["render_offset"].as<std::vector<Scalar>>();
  unity_render_offset_ = Vector<3>(render_offset.data());
  return true;
}

bool VisionEnv::configDynamicObjects(const std::string &yaml_file) {
  //
  if (!(file_exists(yaml_file))) {
    logger_.error("Configuration file %s does not exists.", yaml_file);
    return false;
  }
  YAML::Node cfg_node = YAML::LoadFile(yaml_file);

  // logger_.info("Configuring dynamic objects");

  int num_objects = cfg_node["N"].as<int>();
  // create dynamic objects
  for (int i = 0; i < num_objects; i++) {
    std::string object_id = "Object" + std::to_string(i + 1);
    std::string prefab_id = cfg_node[object_id]["prefab"].as<std::string>();
    std::shared_ptr<UnityObject> obj =
      std::make_shared<UnityObject>(object_id, prefab_id);

    // load location, rotation and size
    std::vector<Scalar> posvec =
      (cfg_node[object_id]["position"]).as<std::vector<Scalar>>();
    std::vector<Scalar> rotvec =
      (cfg_node[object_id]["rotation"]).as<std::vector<Scalar>>();
    std::vector<Scalar> scalevec =
      (cfg_node[object_id]["scale"]).as<std::vector<Scalar>>();
    std::vector<Scalar> boundingboxvec = (cfg_node[object_id]["boundingbox"]).as<std::vector<Scalar>>();

    obj->setPosition(Vector<3>(posvec.data()));
    obj->setRotation(Quaternion(rotvec.data()));
    // actual size in meters
    obj->setSize(Vector<3>(1.0, 1.0, 1.0));
    // scale of the original size
    obj->setScale(Vector<3>(scalevec.data()));
    obj->setBoundingBox(Vector<6>(boundingboxvec.data()));

    std::string csv_name = cfg_node[object_id]["csvtraj"].as<std::string>();
    std::string csv_file = obstacle_cfg_path_ + std::string("/csvtrajs/") +
                           csv_name + std::string(".csv");
    if (!(file_exists(csv_file))) {
      logger_.error("Configuration file %s does not exists.", csv_file);
      return false;
    }
    obj->loadTrajectory(csv_file);

    dynamic_objects_.push_back(obj);
  }
  num_dynamic_objects_ = dynamic_objects_.size();
  return true;
}

bool VisionEnv::configStaticObjects(const std::string &csv_file) {
  //
  if (!(file_exists(csv_file))) {
    logger_.error("Configuration file %s does not exists.", csv_file);
    return false;
  }
  std::ifstream infile(csv_file);
  int i = 0;
  for (auto &row : CSVRange(infile)) {
    // Read column 0 for time
    std::string object_id = "StaticObject" + std::to_string(i + 1);
    std::string prefab_id = (std::string)row[0];

    //
    std::shared_ptr<UnityObject> obj =
      std::make_shared<UnityObject>(object_id, prefab_id);

    //
    Vector<3> pos;
    pos << std::stod((std::string)row[1]), std::stod((std::string)row[2]),
      std::stod((std::string)row[3]);

    Quaternion quat;
    quat.w() = std::stod((std::string)row[4]);
    quat.x() = std::stod((std::string)row[5]);
    quat.y() = std::stod((std::string)row[6]);
    quat.z() = std::stod((std::string)row[7]);

    Vector<3> scale;
    scale << std::stod((std::string)row[8]), std::stod((std::string)row[9]),
      std::stod((std::string)row[10]);

    Vector<6> bounding_box;
    bounding_box << std::stod((std::string)row[11]), std::stod((std::string)row[12]),
      std::stod((std::string)row[13]), std::stod((std::string)row[14]), std::stod((std::string)row[15]), std::stod((std::string)row[16]);

    //
    obj->setPosition(pos);
    obj->setRotation(quat);
    // actual size in meters
    obj->setSize(Vector<3>(1.0, 1.0, 1.0));
    // scale of the original size
    obj->setScale(scale);
    obj->setBoundingBox(bounding_box);
    static_objects_.push_back(obj);
  }
  num_static_objects_ = static_objects_.size();

  return true;
}

bool VisionEnv::configCamera(const YAML::Node &cfg) {
  if (!cfg["rgb_camera"]) {
    logger_.error("Cannot config RGB Camera");
    return false;
  }

  if (!cfg["rgb_camera"]["on"].as<bool>()) {
    logger_.warn("Camera is off. Please turn it on.");
    return false;
  }

  if (quad_ptr_->getNumCamera() >= 1) {
    logger_.warn("Camera has been added. Skipping the camera configuration.");
    return false;
  }

  // create camera
  rgb_camera_ = std::make_shared<RGBCamera>();

  // load camera settings
  std::vector<Scalar> t_BC_vec =
    cfg["rgb_camera"]["t_BC"].as<std::vector<Scalar>>();
  std::vector<Scalar> r_BC_vec =
    cfg["rgb_camera"]["r_BC"].as<std::vector<Scalar>>();

  //
  Vector<3> t_BC(t_BC_vec.data());
  Matrix<3, 3> r_BC =
    (AngleAxis(r_BC_vec[2] * M_PI / 180.0, Vector<3>::UnitZ()) *
     AngleAxis(r_BC_vec[1] * M_PI / 180.0, Vector<3>::UnitY()) *
     AngleAxis(r_BC_vec[0] * M_PI / 180.0, Vector<3>::UnitX()))
      .toRotationMatrix();
  std::vector<bool> post_processing = {false, false, false};
  post_processing[0] = cfg["rgb_camera"]["enable_depth"].as<bool>();
  post_processing[1] = cfg["rgb_camera"]["enable_segmentation"].as<bool>();
  post_processing[2] = cfg["rgb_camera"]["enable_opticalflow"].as<bool>();

  //
  rgb_camera_->setFOV(cfg["rgb_camera"]["fov"].as<Scalar>());
  rgb_camera_->setWidth(cfg["rgb_camera"]["width"].as<int>());
  rgb_camera_->setChannels(cfg["rgb_camera"]["channels"].as<int>());
  rgb_camera_->setHeight(cfg["rgb_camera"]["height"].as<int>());
  rgb_camera_->setRelPose(t_BC, r_BC);
  rgb_camera_->setPostProcessing(post_processing);


  // add camera to the quadrotor
  quad_ptr_->addRGBCamera(rgb_camera_);

  // adapt parameters
  img_width_ = rgb_camera_->getWidth();
  img_height_ = rgb_camera_->getHeight();
  rgb_img_ = cv::Mat::zeros(img_height_, img_width_,
                            CV_MAKETYPE(CV_8U, rgb_camera_->getChannels()));
  depth_img_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
  return true;
}

bool VisionEnv::addQuadrotorToUnity(const std::shared_ptr<UnityBridge> bridge) {
  if (!quad_ptr_) return false;
  bridge->addQuadrotor(quad_ptr_);

  for (int i = 0; i < (int)dynamic_objects_.size(); i++) {
    bridge->addDynamicObject(dynamic_objects_[i]);
  }

  //
  bridge->setRenderOffset(unity_render_offset_);
  bridge->setObjectCSV(static_object_csv_);
  return true;
}

bool VisionEnv::setUnity(bool render) {
  unity_render_ = render;
  if (!unity_render_ || unity_bridge_ptr_ != nullptr) {
    logger_.warn(
      "Unity render is False or Flightmare Bridge has been already created. "
      "Cannot set Unity.");
    return false;
  }
  // create unity bridge
  unity_bridge_ptr_ = UnityBridge::getInstance();
  // add objects to Unity

  addQuadrotorToUnity(unity_bridge_ptr_);

  logger_.info("Flightmare Bridge created.");
  return true;
}

bool VisionEnv::connectUnity(void) {
  if (unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

FrameID VisionEnv::updateUnity(const FrameID frame_id) {
  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(frame_id);
    return unity_bridge_ptr_->handleOutput(frame_id);
  } else {
    return 0;
  }
}

void VisionEnv::disconnectUnity(void) {
  if (unity_bridge_ptr_ != nullptr) {
    unity_bridge_ptr_->disconnectUnity();
    unity_ready_ = false;
  } else {
    logger_.warn("Flightmare Unity Bridge is not initialized.");
  }
}

int VisionEnv::getNumDetectedObstacles(void) { return num_detected_obstacles_; }

std::ostream &operator<<(std::ostream &os, const VisionEnv &vision_env) {
  os.precision(3);
  os << "Vision Environment:\n"
     << "obs dim =            [" << vision_env.obs_dim_ << "]\n"
     << "act dim =            [" << vision_env.act_dim_ << "]\n"
     << "#dynamic objects=    [" << vision_env.num_dynamic_objects_ << "]\n"
     << "#static objects=     [" << vision_env.num_static_objects_ << "]\n"
     << "obstacle dim =       [" << vision_env.num_detected_obstacles_ << "]\n"
     << "sim dt =             [" << vision_env.sim_dt_ << "]\n"
     << "max_t =              [" << vision_env.max_t_ << "]\n"
     << "act_mean =           [" << vision_env.act_mean_.transpose() << "]\n"
     << "act_std =            [" << vision_env.act_std_.transpose() << "]\n"
     << "obs_mean =           [" << vision_env.obs_mean_.transpose() << "]\n"
     << "obs_std =            [" << vision_env.obs_std_.transpose() << "]"
     << std::endl;
  os.precision();
  return os;
}

}  // namespace flightlib