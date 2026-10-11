// World plugin that turns Gazebo's physics off: fake_sim sets the robots' poses directly, and
// sensors (e.g. the Livox lidar) keep updating. Loaded by worlds/*.world as libdisable_physics.so.
// From MIT-ACL's acl-gazebo (acl_sim/src/disable_physics.cpp), as also used in panther and puma.
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class DisablePhysics : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr /*sdf*/) override
  {
    world_ = world;
    update_connection_ =
        event::Events::ConnectWorldUpdateBegin(std::bind(&DisablePhysics::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    if (world_->PhysicsEnabled())
    {
      gzmsg << "disable_physics: turning physics off\n";
      world_->SetPhysicsEnabled(false);
    }
  }

  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
};
GZ_REGISTER_WORLD_PLUGIN(DisablePhysics)
}  // namespace gazebo
