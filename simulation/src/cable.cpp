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
    stiffness_   = sdf->Get<double>("stiffness",   150.0).first;
    damping_     = sdf->Get<double>("damping",     20.0).first;

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

    // Cable is slack — no force
    if (current_length <= rest_length_)
      return;

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

  double rest_length_ = 2.0;
  double stiffness_   = 150.0;
  double damping_     = 20.0;

  // Find the canonical (base) link of a named model
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
