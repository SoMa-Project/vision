#include <limits>

#include <ecto/ecto.hpp>

#include <pcl/common/eigen.h>
#include <pcl/ModelCoefficients.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

using namespace ecto;

namespace ecto_rbo_grasping
{

struct BroadcastDiskGrasps {

	typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;

	spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > plane_coefficients_;
	spore<std::vector< UnalignedVector4f> > plane_centroids_;

	::ros::NodeHandle nh_;
	spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > disks_; // x, y, z, normal_x, normal_y, normal_z, radius
	spore<std::vector< double> > disk_qualities_;
	spore<std::vector< int> > disk_ids_;

	spore<std_msgs::Header> header_;

	spore<pregrasp_msgs::GraspStrategyArray> pregrasp_messages_;

	::ros::Time last_marker_message_;

	static void declare_params(ecto::tendrils& params) {
	}

	static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
		inputs.declare< ::std::vector< ::pcl::ModelCoefficientsConstPtr> > ("models", "Disk coefficients: x, y, z, normal_x, normal_y, normal_z, radius");
		inputs.declare< ::std::vector< int> > ("model_ids", "Disk ids").required(false);
		inputs.declare< ::std::vector< double> >("model_qualities", "");
		inputs.declare< ::std::vector< ::pcl::ModelCoefficientsConstPtr> > ("planes", "planes that should be avoided").required(false);
		inputs.declare< ::std::vector< UnalignedVector4f> > ("plane_centroids", "planes that should be avoided").required(false);
                inputs.declare< std_msgs::Header>("header", "header of the sensed data that was used to create the grasps");

                outputs.declare<pregrasp_msgs::GraspStrategyArray>("pregrasp_messages", "");
	}

	void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) {
		disks_ = inputs["models"];
		disk_ids_ = inputs["model_ids"];
		disk_qualities_ = inputs["model_qualities"];
		plane_coefficients_ = inputs["planes"];
		plane_centroids_ = inputs["plane_centroids"];
		header_ = inputs["header"];

		pregrasp_messages_ = outputs["pregrasp_messages"];

		last_marker_message_ = ::ros::Time::now();
	}

	bool considerObstacles(Eigen::Vector3f& centroid, Eigen::Matrix3f& orientation, Eigen::Vector3f cyl_axis, bool changes, pregrasp_msgs::GraspStrategy& grasp) {
		changes = false;
		double min_distance = std::numeric_limits<double>::max();

		for (size_t i = 0; i < plane_coefficients_->size(); ++i) {
			::pcl::ModelCoefficientsConstPtr p = plane_coefficients_->at(i);
			Eigen::Vector3f normal(p->values[0], p->values[1], p->values[2]);

			// check free space along disk axis
			// distance between cylinder and plane
			double distance = centroid[0] * p->values[0] + centroid[1] * p->values[1] + centroid[2] * p->values[2] + p->values[3];

			min_distance = std::min(min_distance, distance);

			if (distance > 0.0 && distance < 0.1) {
				// calculate point on the cylinder axis which obeys distance threshold
				double denominator = cyl_axis[0] * p->values[0] + cyl_axis[1] * p->values[1] + cyl_axis[2] * p->values[2];

				// if plane is parallel to cylinder axis do something
				if (denominator < 1e-3)
					return false;

				double alpha = -(distance - 0.1) / denominator;

//				if (std::fabs(alpha) < height_2) {
//
//					ROS_INFO("WOULD CHANGE!");
//					changes = true;
////					centroid = centroid + alpha * cyl_axis;
//				}
			}

			// check that camera fits (prefer orientation which keeps camera away from plane)
			if (orientation.col(0).dot(normal) > 0.0) {
				// flip
				ROS_INFO("FLIPPING!");
				changes = true;
				orientation.col(0) *= -1.0;
				orientation.col(1) *= -1.0;
			}

			// check along approach direction (maybe: make approach comply with plane normal)
		}

		grasp.quality_grasp = min_distance;

		return true;
	}

	void publishRVizPoses(pregrasp_msgs::GraspStrategyArray& grasps) {
		static ros::Publisher pose_publisher = nh_.advertise< ::geometry_msgs::PoseStamped>("/grasps", 10);

		::geometry_msgs::PoseStamped msg;
		msg.header = grasps.header;
		for (std::vector< pregrasp_msgs::GraspStrategy >::const_iterator it = grasps.strategies.begin(); it != grasps.strategies.end(); ++it) {
			msg.pose = it->pregrasp_pose.pose.pose;

			pose_publisher.publish(msg);
		}
	}

	void publishRVizMarkers(pregrasp_msgs::GraspStrategyArray& grasps) {
		static ros::Publisher marker_publisher = nh_.advertise< ::visualization_msgs::MarkerArray>("/disk_grasps_marker", 10);

		::visualization_msgs::MarkerArray msgs;
		::visualization_msgs::Marker msg;
		msg.header = grasps.header;
		msg.type = ::visualization_msgs::Marker::MESH_RESOURCE;
		msg.action = ::visualization_msgs::Marker::ADD;
		msg.lifetime = ::ros::Time::now() - last_marker_message_;

		msg.scale.x = 0.01;
		msg.scale.y = 0.01;
		msg.scale.z = 0.01;

		msg.color.b = 1.0;
		msg.color.r = msg.color.g = 0;
		msg.color.a = 0.7;

		msg.mesh_resource = "package://barrett_hand_262/data/barrett_spherical.dae";

		static int id = 0;
		for (std::vector< pregrasp_msgs::GraspStrategy >::const_iterator it = grasps.strategies.begin(); it != grasps.strategies.end(); ++it) {
			msg.id = id++;
			msg.pose = it->pregrasp_pose.pose.pose;

			msgs.markers.push_back(msg);
		}

		marker_publisher.publish(msgs);
		last_marker_message_ = ::ros::Time::now();
	}

	int process(const tendrils& /*inputs*/, const tendrils& /*outputs*/) {
		// create ros grasp messages
		pregrasp_msgs::GraspStrategyArray disk_grasps;
		disk_grasps.header = *header_;

		// iterate over all hypotheses
		for (std::vector< ::pcl::ModelCoefficientsConstPtr>::const_iterator it = disks_->begin(); it != disks_->end(); ++it) {
			Eigen::Vector3f centroid((*it)->values[0], (*it)->values[1], (*it)->values[2]);
			Eigen::Vector3f centroid_normalized = centroid.normalized();
			Eigen::Vector3f direction(-(*it)->values[3], -(*it)->values[4], -(*it)->values[5]);
			Eigen::Vector3f direction_normalized = direction.normalized();
			float radius = (*it)->values[6];

			float offset = 0;//.065; // that's just for 3d visualization; better: change origin in model to palm!

			// create an approach vector plus roll angle: cyl_axis X view_y
			pregrasp_msgs::GraspStrategy grasp;
			grasp.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_DISK;
			grasp.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE;
			static int cnt = 0;
			grasp.id = cnt++;//disk_ids_->at(it - disks_->begin());

                        Eigen::Vector3f approach_x = direction_normalized.cross(centroid_normalized);
                        Eigen::Vector3f approach_z = -direction_normalized;
                        Eigen::Vector3f approach_y = approach_z.cross(approach_x);

			Eigen::Matrix3f rotation;
			rotation << approach_x, approach_y, approach_z;

			bool changes = false;
//			bool collision = considerObstacles(centroid, rotation, direction_normalized, changes, grasp);
			bool collision = false;

			if (changes) {
				ROS_INFO("Grasps were changed!");
			}

			if (collision) {
				ROS_WARN("Disk Grasp ignored due to collisions.");
			}
			else {
				Eigen::Quaternionf q(rotation);
				grasp.pregrasp_pose.pose.pose.orientation.x = q.x();
				grasp.pregrasp_pose.pose.pose.orientation.y = q.y();
				grasp.pregrasp_pose.pose.pose.orientation.z = q.z();
				grasp.pregrasp_pose.pose.pose.orientation.w = q.w();

				Eigen::Vector3f center_offset = centroid - offset * approach_z.normalized();
				grasp.pregrasp_pose.pose.pose.position.x = center_offset[0];
				grasp.pregrasp_pose.pose.pose.position.y = center_offset[1];
				grasp.pregrasp_pose.pose.pose.position.z = center_offset[2];

				grasp.pregrasp_pose.pose.header = *header_;

			        grasp.quality_grasp = (*disk_qualities_)[it - disks_->begin()];

				disk_grasps.strategies.push_back(grasp);
			}
		}

		publishRVizPoses(disk_grasps);
		publishRVizMarkers(disk_grasps);

		static ros::Publisher disk_grasps_publisher = nh_.advertise< pregrasp_msgs::GraspStrategyArray>("/disk_grasps", 10);
		disk_grasps_publisher.publish(disk_grasps);

		(*pregrasp_messages_) = disk_grasps;

		return OK;
	}
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::BroadcastDiskGrasps, "BroadcastDiskGrasps", "Broadcast some disk grasps.");
