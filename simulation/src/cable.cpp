#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <sdf/sdf.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/transport/Node.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/pose.pb.h>

#include <iostream>
#include <optional>

using namespace gz;
using namespace sim;

class CablePlugin
  : public System,
    public ISystemConfigure,
    public ISystemUpdate
{
public:
  void Configure(const Entity &/*entity*/,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &/*ecm*/,
                 EventManager &/*eventMgr*/) override
  {
    rest_length_ = sdf->Get<double>("rest_length", 2.2).first;
    stiffness_   = sdf->Get<double>("stiffness",   50.0).first;
    damping_     = sdf->Get<double>("damping",     30.0).first;

    std::cout << "[CablePlugin] Loaded."
              << " rest_length=" << rest_length_
              << " k=" << stiffness_
              << " d=" << damping_ << std::endl;
  }

  void Update(const UpdateInfo &/*info*/,
              EntityComponentManager &ecm) override
  {
    // --- Find drone links on first tick (deferred until both are spawned) ---
    if (!links_found_)
    {
      link0_ = FindCanonicalLink(ecm, "x500_0");
      link1_ = FindCanonicalLink(ecm, "x500_1");

      if (link0_ == kNullEntity || link1_ == kNullEntity)
        return;

      // Enable velocity tracking
      gz::sim::Link l0(link0_);
      gz::sim::Link l1(link1_);
      l0.EnableVelocityChecks(ecm, true);
      l1.EnableVelocityChecks(ecm, true);

      links_found_ = true;
      std::cout << "[CablePlugin] Both drone links found. Cable active." << std::endl;
    }

    // --- Read poses via Link helper ---
    gz::sim::Link l0(link0_);
    gz::sim::Link l1(link1_);

    auto pose0_opt = l0.WorldPose(ecm);
    auto pose1_opt = l1.WorldPose(ecm);
    if (!pose0_opt.has_value() || !pose1_opt.has_value()) return;

    math::Vector3d pos0 = pose0_opt->Pos();
    math::Vector3d pos1 = pose1_opt->Pos();

    // --- Read velocities ---
    auto vel0_opt = l0.WorldLinearVelocity(ecm);
    auto vel1_opt = l1.WorldLinearVelocity(ecm);
    math::Vector3d v0 = vel0_opt.has_value() ? *vel0_opt : math::Vector3d::Zero;
    math::Vector3d v1 = vel1_opt.has_value() ? *vel1_opt : math::Vector3d::Zero;

    // --- Cable geometry ---
    math::Vector3d delta = pos1 - pos0;
    double current_length = delta.Length();

    // --- Update visual marker every tick ---
    UpdateCableMarker(pos0, pos1);

    // Cable is slack — no force
    if (current_length <= rest_length_)
    {
      ApplyForce(ecm, link0_, math::Vector3d::Zero);
      ApplyForce(ecm, link1_, math::Vector3d::Zero);
      return;
    }

    math::Vector3d unit = delta / current_length;

    // --- Spring-damper force (cable only pulls, never pushes) ---
    double stretch   = current_length - rest_length_;
    double rel_vel   = (v1 - v0).Dot(unit);
    double force_mag = stiffness_ * stretch + damping_ * rel_vel;
    if (force_mag < 0.0) force_mag = 0.0;

    math::Vector3d force_on_0 =  unit * force_mag;
    math::Vector3d force_on_1 = -unit * force_mag;

    // --- Apply forces ---
    ApplyForce(ecm, link0_, force_on_0);
    ApplyForce(ecm, link1_, force_on_1);
  }

private:
  Entity link0_ = kNullEntity;
  Entity link1_ = kNullEntity;
  bool links_found_ = false;
  gz::transport::Node transport_node_;

  double rest_length_ = 2.0;
  double stiffness_   = 150.0;
  double damping_     = 20.0;

  // ---------------------------------------------------
  // Visual: draw a red line between the two drone links
  // ---------------------------------------------------
  void UpdateCableMarker(const math::Vector3d &p0, const math::Vector3d &p1)
{
  gz::msgs::Marker marker;
  marker.set_ns("cable");
  marker.set_id(1);
  marker.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker.set_type(gz::msgs::Marker::CYLINDER);

  marker.mutable_lifetime()->set_sec(0);
  marker.mutable_lifetime()->set_nsec(100000000);

  // Midpoint position
  math::Vector3d mid = (p0 + p1) / 2.0;

  // Length of cable
  double length = (p1 - p0).Length();

  // Orientation: cylinder Z-axis must align with cable direction
  math::Vector3d dir = (p1 - p0).Normalized();
  math::Vector3d z_axis(0, 0, 1);
  math::Quaterniond rot;
  rot.From2Axes(z_axis, dir);

  // Set pose (position + orientation)
  gz::msgs::Set(marker.mutable_pose(),
    math::Pose3d(mid, rot));

  // Set scale: x/y = diameter, z = length
  gz::msgs::Set(marker.mutable_scale(),
    math::Vector3d(0.02, 0.02, length));  // 2cm diameter

  // Yellow cable
  auto *mat = marker.mutable_material();
  mat->mutable_ambient()->set_r(1.0);
  mat->mutable_ambient()->set_g(0.8);
  mat->mutable_ambient()->set_b(0.0);
  mat->mutable_ambient()->set_a(1.0);
  mat->mutable_diffuse()->set_r(1.0);
  mat->mutable_diffuse()->set_g(0.8);
  mat->mutable_diffuse()->set_b(0.0);
  mat->mutable_diffuse()->set_a(1.0);

  transport_node_.Request("/marker", marker);
}

  // ---------------------------------------------------
  // Find the canonical (base) link of a named model
  // ---------------------------------------------------
  Entity FindCanonicalLink(EntityComponentManager &ecm,
                           const std::string &model_name)
  {
    Entity model_entity = kNullEntity;

    ecm.Each<components::Model, components::Name>(
      [&](const Entity &entity,
          const components::Model *,
          const components::Name *name) -> bool
      {
        if (name->Data() == model_name)
        {
          model_entity = entity;
          return false;
        }
        return true;
      });

    if (model_entity == kNullEntity)
      return kNullEntity;

    gz::sim::Model model(model_entity);
    return model.CanonicalLink(ecm);
  }

  // ---------------------------------------------------
  // Apply a world-frame force to a link via wrench cmd
  // ---------------------------------------------------
  void ApplyForce(EntityComponentManager &ecm,
                  Entity link,
                  const math::Vector3d &force)
  {
    auto *wrench = ecm.Component<components::ExternalWorldWrenchCmd>(link);
    if (!wrench)
    {
      ecm.CreateComponent(link, components::ExternalWorldWrenchCmd());
      wrench = ecm.Component<components::ExternalWorldWrenchCmd>(link);
    }

    msgs::Wrench wrench_msg;
    wrench_msg.mutable_force()->set_x(force.X());
    wrench_msg.mutable_force()->set_y(force.Y());
    wrench_msg.mutable_force()->set_z(force.Z());
    wrench->Data() = wrench_msg;
  }
};

GZ_ADD_PLUGIN(CablePlugin, System, ISystemConfigure, ISystemUpdate)
GZ_ADD_PLUGIN_ALIAS(CablePlugin, "CablePlugin")