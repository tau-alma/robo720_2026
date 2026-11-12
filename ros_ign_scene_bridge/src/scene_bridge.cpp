#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/obstacle_array.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/scene.pb.h>
#include <ignition/msgs/empty.pb.h>

using namespace std::chrono_literals;

class SceneBridge : public rclcpp::Node
{
public:
  SceneBridge() : Node("scene_bridge")
  {
    pub_ = create_publisher<custom_msgs::msg::ObstacleArray>(
        "obstacles", rclcpp::QoS(1).transient_local());

    // Retry the service call until Gazebo is up and the scene is populated.
    retry_timer_ = create_wall_timer(500ms, std::bind(&SceneBridge::tryFetchScene, this));
  }

private:
  void tryFetchScene()
  {
    ignition::msgs::Empty req;
    ignition::msgs::Scene rep;
    bool result = false;

    bool ok = gz_node_.Request(
        "/world/obstacle_world/scene/info", req, 2000, rep, result);

    if (!ok || !result || rep.model_size() == 0)
    {
      RCLCPP_WARN(get_logger(), "Scene service not ready yet, retrying...");
      return;
    }

    publishObstacles(rep);
    retry_timer_->cancel();  // got it, no need to keep polling
  }

  void publishObstacles(const ignition::msgs::Scene &scene)
  {
    custom_msgs::msg::ObstacleArray msg;
    msg.header.frame_id = "world";
    msg.header.stamp = this->get_clock()->now();

    for (const auto &model : scene.model()) {
        for (const auto &link : model.link()) {
            // NOTE: reading geometry from `visual`, not `collision` — the Scene
            // message from SceneBroadcaster doesn't populate collision
            // geometry (it's built for GUI rendering). This is fine as long as
            // our own SDF authoring keeps visual and collision geometry matched
            // for these obstacles.
            for (const auto &visual : link.visual()) {
                const auto &geom = visual.geometry();

                if (geom.type() == ignition::msgs::Geometry::SPHERE) {
                    custom_msgs::msg::Obstacle obs;

                    obs.name = model.name();
                    obs.shape_type = custom_msgs::msg::Obstacle::SPHERE;
                    obs.position.x = model.pose().position().x();
                    obs.position.y = model.pose().position().y();
                    obs.position.z = model.pose().position().z();
                    obs.radius = geom.sphere().radius();

                    msg.obstacles.push_back(obs);

                    RCLCPP_INFO(get_logger(), "Found sphere obstacle '%s' r=%.3f",
                                obs.name.c_str(), geom.sphere().radius());
                }
                else if (geom.type() == ignition::msgs::Geometry::BOX) {
                    if (model.name() == "fr3_arm") {
                        continue;
                    }
                    custom_msgs::msg::Obstacle obs;

                    obs.name = model.name();
                    obs.shape_type = custom_msgs::msg::Obstacle::BOX;
                    obs.position.x = model.pose().position().x();
                    obs.position.y = model.pose().position().y();
                    obs.position.z = model.pose().position().z();
                    obs.orientation.x = model.pose().orientation().x();
                    obs.orientation.y = model.pose().orientation().y();
                    obs.orientation.z = model.pose().orientation().z();
                    obs.orientation.w = model.pose().orientation().w();
                    obs.box_size.at(0) = geom.box().size().x();
                    obs.box_size.at(1) = geom.box().size().y();
                    obs.box_size.at(2) = geom.box().size().z();

                    msg.obstacles.push_back(obs);
                    
                    RCLCPP_INFO(get_logger(), "Found box obstacle '%s' size=[%.3f, %.3f, %.3f]",
                                obs.name.c_str(), obs.box_size.at(0), obs.box_size.at(1), obs.box_size.at(2));
                }
            }
        }
    }

    pub_->publish(msg);
  }

  ignition::transport::Node gz_node_;
  rclcpp::Publisher<custom_msgs::msg::ObstacleArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneBridge>());
  rclcpp::shutdown();
  return 0;
}