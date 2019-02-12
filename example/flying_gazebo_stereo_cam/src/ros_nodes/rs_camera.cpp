#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/icp.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <boost/thread/thread.hpp>

class Rs_Camera{
    private:
        ros::NodeHandle nh;
        ros::Publisher meshPub;
        ros::Subscriber pc_sub;
        unsigned int i = 0; 

    public: 
        void init(){
        	meshPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/flying_gazebo_stereo_cam/mesh", 1);
            pc_sub = nh.subscribe("/world/pcl_input", 1, &Rs_Camera::pointcloud_cb, this);
        }

        void pointcloud_cb(const sensor_msgs::PointCloud2& pcMsg){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::PCLPointCloud2::Ptr cloud_filtered_inter (new pcl::PCLPointCloud2 ()); // new ROS message type 
          
            pcl::fromROSMsg(pcMsg, *cloud);
            
            /*********************Outlier_removal*********************/
            // Point cloud for filtering 
            std::cerr << "Point cloud before filtering: " << std::endl;
            std::cerr << *cloud << std::endl;
            
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter; // outlier removal 
            statFilter.setInputCloud (cloud);
            statFilter.setMeanK (50);
            statFilter.setStddevMulThresh (1.0);
            statFilter.filter (*cloud_filtered);

            std::cerr << "Point Cloud after filtering: " << std::endl;
            std::cerr << *cloud_filtered << std::endl;
            
            /***********************Downsampling***********************/
            // Point cloud for downsampling  
            // std::cerr << "Point Cloud before downsampling: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
            pcl::PCLPointCloud2::Ptr cloud_pcl2 (new pcl::PCLPointCloud2 ());
            pcl::PCLPointCloud2::Ptr cloud_downsampled (new pcl::PCLPointCloud2 ());
            
            pcl::toPCLPointCloud2(*cloud_filtered, *cloud_pcl2); // convert from templated pcl to PCL2
            
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor; // downsampling 
            sor.setInputCloud (cloud_pcl2);
            sor.setLeafSize (0.01f, 0.01f, 0.01f);
            sor.filter (*cloud_downsampled);
            
            std::cerr << "PointCloud after downsampling: " << cloud_downsampled->width * cloud_downsampled->height 
                   << " data points (" << pcl::getFieldsList (*cloud_downsampled) << ").";
            
            /************************Write_to_PCD*********************/
            /*
            pcl::PCDWriter writer;
            writer.write ("table_scene_rs300_downsampled.pcd", *cloud_downsampled, // write to PCD format
            Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
            */ 
            
            // Convert to the templated point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ready (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2 (*cloud_downsampled, *cloud_ready);
            
            /***********************Cloud_visualization***************/
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_view (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::copyPointCloud(*cloud_ready, *cloud_view);
             
            pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer"); // visualization 
            viewer.showCloud (cloud_view);
            while (!viewer.wasStopped ())
            {
            }
            
            /***********************Write_PointCloud_to_PCD***********/
            std::ostringstream oss;
            oss << std::setfill( '0' ) << std::setw( 3 ) << i++;
            std::cout << oss.str() + ".pcd" << std::endl;
             
            // Save Point Cloud to PCD File
            std::string writePath = "/home/helen/catkin_ws/src/rpg_ig_active_reconstruction/example/flying_gazebo_stereo_cam/src/ros_nodes";
            pcl::io::savePCDFile( oss.str() + ".pcd", *cloud_ready );
            
            /***********************ICP_registration******************/
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>); // reset? 
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);
            
            cloud_icp = cloud_ready; // get one point cloud from callback 
            // Remove NAN points from point cloud
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_icp,*cloud_icp, indices);
            
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(cloud_icp);
            pcl::PointCloud<pcl::PointXYZ> cloud_tmp;
            
            int n = Final->points.size();
            if(n = 0) {
            	*Final+=cloud_tmp; 
            } 
            else {
               icp.setInputTarget(Final);
            	            
               icp.setMaxCorrespondenceDistance(5);
               icp.setMaximumIterations(100);
               icp.setTransformationEpsilon (1e-12);
               icp.setEuclideanFitnessEpsilon(0.1);
            	            
               //pcl::PointCloud<pcl::PointXYZ> cloud_tmp;
               icp.align(cloud_tmp);
            	            
               std::cout << "has converged:" << icp.hasConverged() << " score: " <<
               icp.getFitnessScore() << std::endl;
               std::cout << icp.getFinalTransformation() << std::endl;
               *Final = cloud_tmp;
            }
            /*
            icp.setInputTarget(Final);
            
            icp.setMaxCorrespondenceDistance(5);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon (1e-12);
            icp.setEuclideanFitnessEpsilon(0.1);
            
            pcl::PointCloud<pcl::PointXYZ> cloud_tmp;
            icp.align(cloud_tmp);
            
            std::cout << "has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;
            *Final = cloud_tmp; 
            */ 
            
            // Convert the pcl::PointCloud to sensor_msgs/PointCloud2
            sensor_msgs::PointCloud2 mesh;
            pcl::toROSMsg( *Final, mesh );
            // Publish the mesh
            meshPub.publish(mesh);
            
            
        } 

};

int main(int argc, char **argv){
    ros::init(argc, argv, "rs_camera");
    Rs_Camera camera_pc;
    camera_pc.init();
    ros::Rate loop_rate(10);
	ROS_INFO("Hello");

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

