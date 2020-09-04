#include "active_3d_planning_fiesta/map/fiesta_interface.h"
#include "active_3d_planning_core/data/system_constraints.h"


namespace active_3d_planning {
    namespace map {

        ModuleFactoryRegistry::Registration<FiestaMap> FiestaMap::registration("FiestaMap");

        FiestaMap::FiestaMap(PlannerI &planner) : OccupancyMap(planner) {}

        void FiestaMap::setupFromParamMap(Module::ParamMap *param_map) {
            // create an esdf server
            ros::NodeHandle nh("~/map");
            fiesta_server_.reset(new fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>(nh));

        }

        bool FiestaMap::isTraversable(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
            if (isObserved(position)){
                double distance = fiesta_server_->esdf_map_->GetDistance(position);
                if (distance == fiesta_server_->esdf_map_->infinity_){ 
                    return false;
                }
                
                return (distance > planner_.getSystemConstraints().collision_radius);
            }
            else{
                return false;
            }
        }

        bool FiestaMap::isObserved(const Eigen::Vector3d &point) { 
            Eigen::Vector3i vox;
            // Get voxel corresponding to point
            fiesta_server_->esdf_map_->Pos2Vox(point,vox);
            int id = fiesta_server_->esdf_map_->Vox2Idx(vox);
            if (fiesta_server_->esdf_map_->distance_buffer_[id] == fiesta_server_->esdf_map_->undefined_)
                return false;
            else
                return true;
        }

        // get occupancy
        unsigned char FiestaMap::getVoxelState(const Eigen::Vector3d &point) { 
            // Fiesta and the planner have different definitions of occupied voxels 
            int occupancy = fiesta_server_->esdf_map_->GetOccupancy(point); // this returns only true or false, need to check distance buffer to know if unknown 
            Eigen::Vector3i vox;
            // Get voxel corresponding to point
            fiesta_server_->esdf_map_->Pos2Vox(point,vox);
            int id = fiesta_server_->esdf_map_->Vox2Idx(vox);
            // Only way to check for unseen voxel is through the distance buffer of the ESDF 
            double distance = fiesta_server_->esdf_map_->distance_buffer_[id];

            if (distance == fiesta_server_->esdf_map_->undefined_){ 
                return FiestaMap::UNKNOWN; // 2
            } 

            else if (occupancy == 1){
                return FiestaMap::OCCUPIED; // 0
            } 

            else if (occupancy == 0){
                return FiestaMap::FREE; // 1
            } 

            else{
                ROS_ERROR("Error in getVoxelState, unexpected behavior of esdf_map->GetOccupancy");
                return 0;
            }
        }

        // get voxel size
        double FiestaMap::getVoxelSize() {
            return fiesta_server_->parameters_.resolution_;
        }

        // get the center of a voxel from input point
        bool FiestaMap::getVoxelCenter(Eigen::Vector3d *center, const Eigen::Vector3d &point) {
            Eigen::Vector3i vox;
            // Get voxel corresponding to point
            fiesta_server_->esdf_map_->Pos2Vox(point,vox);
            // Get center of corresponding voxel
            fiesta_server_->esdf_map_->Vox2Pos(vox,*center);
            return true;

        }

    } // namespace map
} // namespace active_3d_planning
