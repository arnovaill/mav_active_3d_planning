#ifndef ACTIVE_3D_PLANNING_FIESTA_MAP_FIESTA_H
#define ACTIVE_3D_PLANNING_FIESTA_MAP_FIESTA_H

#include "active_3d_planning_core/map/occupancy_map.h"
#include <active_3d_planning_core/module/module_factory_registry.h>

#include "Fiesta.h"
#include <memory>

namespace active_3d_planning {
    namespace map {

        // Voxblox as a map representation
        class FiestaMap : public OccupancyMap { // change this here
        public:
            FiestaMap(PlannerI &planner);

            // implement virtual methods
            void setupFromParamMap(Module::ParamMap *param_map) override;

            // check collision for a single pose
            virtual bool isTraversable(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) override;

            // check whether point is part of the map
            virtual bool isObserved(const Eigen::Vector3d &point) override;

            // get occupancy
            virtual unsigned char getVoxelState(const Eigen::Vector3d &point) override;

            // get voxel size
            virtual double getVoxelSize() override;

            // get the center of a voxel from input point
            virtual bool getVoxelCenter(Eigen::Vector3d *center, const Eigen::Vector3d &point) override;

            // // get the stored distance
            // virtual double getVoxelDistance(const Eigen::Vector3d &point) override;

            // // get the stored weight
            // virtual double getVoxelWeight(const Eigen::Vector3d &point) override;

            // // get the maximum allowed weight (return 0 if using uncapped weights)
            // virtual double getMaximumWeight() override;

            // // accessor to the server for specialized planners
            // voxblox::EsdfServer &getESDFServer();

        protected:

            static ModuleFactoryRegistry::Registration<FiestaMap> registration;
            std::unique_ptr<fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>> fiesta_server_;


            // esdf server that conode_ntains the map, subscribe to external ESDF/TSDF updates
            //std::unique_ptr<voxblox::EsdfServer> esdf_server_;
            
            // cache constants
            //double c_voxel_size_;
            //double c_block_size_;
            //double c_maximum_weight_;
        };

    } // namespace map
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_FIESTA_MAP_FIESTA_H
