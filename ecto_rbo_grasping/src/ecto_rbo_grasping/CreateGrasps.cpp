#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>

#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>


typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;


namespace ecto_rbo_grasping
{

struct CreateGrasps
{
  ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_;

  ecto::spore<std::vector<UnalignedVector4f> > positions_;
  ecto::spore<std::vector<UnalignedVector3f> > approach_vectors_;
  ecto::spore<std::vector<UnalignedVector3f> > roll_vectors_;
  ecto::spore<std_msgs::Header> header_;

  ecto::spore<int> pregrasp_configuration_;
  ecto::spore<int> strategy_;
  ecto::spore<double> roll_offset_;

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<int>("pregrasp_configuration", "", pregrasp_msgs::GraspStrategy::PREGRASP_SPHERE);
    params.declare<int>("strategy", "", pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE);
    
    params.declare<double>("roll_offset", "Rotate by this amount along the z-axis.", 0.0);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    inputs.declare<std_msgs::Header>("header", "The frame_id of this header is used as the frame_id of the published grasps.");
    inputs.declare<std::vector<UnalignedVector4f> >("positions", "Positions.");
    inputs.declare<std::vector<UnalignedVector3f> >("approach_vectors", "The approach vector of the grasp (which is usually normal to the palm).");
    inputs.declare<std::vector<UnalignedVector3f> >("roll_vectors", "Defines the roll angle around the wrist.").required(false);

    outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages", "All the grasps that should be used.");
  }

  void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    pregrasp_configuration_ = params["pregrasp_configuration"];
    strategy_ = params["strategy"];
    roll_offset_ = params["roll_offset"];
    
    header_ = inputs["header"];
    positions_ = inputs["positions"];
    approach_vectors_ = inputs["approach_vectors"];
    roll_vectors_ = inputs["roll_vectors"];

    pregrasp_messages_ = outputs["pregrasp_messages"];
  }

  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    pregrasp_msgs::GraspStrategyArrayPtr pregrasp_messages(new ::pregrasp_msgs::GraspStrategyArray);
    pregrasp_messages->header = *header_;
    
    // generate grasps also if number of provided arrays is not equal
    int grasp_count = std::max(positions_->size(), approach_vectors_->size());
    int position_increment = 1;
    int approach_vector_increment = 1;
    if (positions_->size() != approach_vectors_->size())
    {
      ROS_INFO("Vectors of positions and approach vectors have different sizes! %zu vs %zu \n Will reuse some.", positions_->size(), approach_vectors_->size());
      
      if (positions_->empty() || approach_vectors_->empty())
      {
          (*pregrasp_messages_) = pregrasp_messages;
          return ecto::OK;
      }
      
      position_increment = positions_->size() / grasp_count;
      approach_vector_increment = approach_vectors_->size() / grasp_count;
    }

    int pos_cnt = 0;
    int app_cnt = 0;
    for (size_t i = 0; i < grasp_count; ++i)
    {
      pregrasp_msgs::GraspStrategy grasp;
      grasp.pregrasp_configuration = *pregrasp_configuration_;
      grasp.strategy = *strategy_;
      grasp.id = i;

      Eigen::Vector3f approach_z = (*approach_vectors_)[app_cnt];
      Eigen::Vector3f approach_x;
      if (roll_vectors_.user_supplied() && !roll_vectors_->empty())
      {
          // ensure that roll vector points towards camera frame
          if (roll_vectors_->at(0).dot(Eigen::Vector3f::UnitZ()) > 0)
            approach_x = approach_z.cross(roll_vectors_->at(0));
          else
            approach_x = approach_z.cross(-roll_vectors_->at(0));
      }
      else
      {
          approach_x = approach_z.cross((*positions_)[app_cnt].head<3>().normalized());
      }
      Eigen::Vector3f approach_y = approach_z.cross(approach_x);
      Eigen::Matrix3f rotation;
      rotation << approach_x, approach_y, approach_z;
      Eigen::Quaternionf q(rotation);
      
      q = q * Eigen::AngleAxisf(*roll_offset_, Eigen::Vector3f::UnitZ());
      
      grasp.pregrasp_pose.pose.header = pregrasp_messages->header;
      grasp.pregrasp_pose.pose.pose.orientation.x = q.x();
      grasp.pregrasp_pose.pose.pose.orientation.y = q.y();
      grasp.pregrasp_pose.pose.pose.orientation.z = q.z();
      grasp.pregrasp_pose.pose.pose.orientation.w = q.w();

      //Eigen::Vector3f center_offset = centroid - radius * rotation.col(2); //approach_z.normalized();
      grasp.pregrasp_pose.pose.pose.position.x = (*positions_)[pos_cnt][0];
      grasp.pregrasp_pose.pose.pose.position.y = (*positions_)[pos_cnt][1];
      grasp.pregrasp_pose.pose.pose.position.z = (*positions_)[pos_cnt][2];

      grasp.pregrasp_pose.pose.header = *header_;

      grasp.quality_grasp = grasp.quality_approach = grasp.quality_closing = 1.0;

      pregrasp_messages->strategies.push_back(grasp);
      
      pos_cnt += position_increment;
      app_cnt += approach_vector_increment;
    }

    (*pregrasp_messages_) = pregrasp_messages;
    
    return ecto::OK;
  }
};


struct CreateGraspFromTF
{
  ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_;

  ecto::spore<UnalignedAffine3f> transform_;
  ecto::spore<std_msgs::Header> header_;

  ecto::spore<int> pregrasp_configuration_;
  ecto::spore<int> strategy_;

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<int>("pregrasp_configuration", "", pregrasp_msgs::GraspStrategy::PREGRASP_SPHERE);
    params.declare<int>("strategy", "", pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    inputs.declare<std_msgs::Header>("header", "The frame_id of this header is used as the frame_id of the published grasps.");
    inputs.declare<UnalignedAffine3f>("transform", "Transform.");

    outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_message", "One grasp.");
  }

  void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    pregrasp_configuration_ = params["pregrasp_configuration"];
    strategy_ = params["strategy"];
    
    header_ = inputs["header"];
    transform_ = inputs["transform"];
    
    pregrasp_messages_ = outputs["pregrasp_message"];
  }

  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    pregrasp_msgs::GraspStrategyArrayPtr pregrasp_messages(new ::pregrasp_msgs::GraspStrategyArray);
    pregrasp_messages->header = *header_;
    
    pregrasp_msgs::GraspStrategy grasp;
    grasp.pregrasp_configuration = *pregrasp_configuration_;
    grasp.strategy = *strategy_;
    grasp.id = 0;

    Eigen::Quaternionf q(transform_->linear());
    grasp.pregrasp_pose.pose.pose.orientation.x = q.x();
    grasp.pregrasp_pose.pose.pose.orientation.y = q.y();
    grasp.pregrasp_pose.pose.pose.orientation.z = q.z();
    grasp.pregrasp_pose.pose.pose.orientation.w = q.w();

      
    grasp.pregrasp_pose.pose.pose.position.x = transform_->translation()[0];
    grasp.pregrasp_pose.pose.pose.position.y = transform_->translation()[1];
    grasp.pregrasp_pose.pose.pose.position.z = transform_->translation()[2];

    grasp.pregrasp_pose.pose.header = *header_;

    grasp.quality_grasp = grasp.quality_approach = grasp.quality_closing = 1.0;
    pregrasp_messages->strategies.push_back(grasp);
    
    (*pregrasp_messages_) = pregrasp_messages;
    
    return ecto::OK;
  }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::CreateGrasps, "CreateGrasps", "Create pregrasp messages from 3D position and a single normal vectors (= approach vector).");
ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::CreateGraspFromTF, "CreateGraspFromTF", "Create pregrasp message from transform.");
