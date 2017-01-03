// C��min,max�}�N���𖳌��ɂ���
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <Windows.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d.h>

// obj�f�[�^���擾
const char filename[] = "bunny.obj";

// �_�Q�̌^���`
typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGBNormal NormalType;

void main()
{
	try{
		// obj�t�@�C����ǂݍ���
		pcl::PolygonMesh::Ptr mesh( new pcl::PolygonMesh() );
		pcl::PointCloud<PointType>::Ptr obj_pcd( new pcl::PointCloud<PointType> );
		if ( pcl::io::loadPolygonFileOBJ( filename, *mesh ) != -1 ){
			pcl::fromPCLPointCloud2( mesh->cloud, *obj_pcd );
		}

		// Create Cloud Viewer
		pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );
		// �o��
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals( new pcl::PointCloud<pcl::Normal> );

		//... read, pass in or create a point cloud ...
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<PointType, pcl::Normal> ne;
		ne.setInputCloud( obj_pcd );

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<PointType>::Ptr tree( new pcl::search::KdTree<PointType>() );
		ne.setSearchMethod( tree );

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch( 0.03 );

		// Compute the features
		ne.compute( *cloud_normals );

		// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

		viewer.addPointCloud( obj_pcd, "cloud" );
		viewer.addPointCloudNormals<PointType, pcl::Normal>( obj_pcd, cloud_normals, 100, -0.05, "normals" );

		while ( !viewer.wasStopped() ) {
			// �X�N���[�����X�V����
			viewer.spinOnce();

			// ESCAPE�L�[�������ꂽ��I��
			if ( GetKeyState( VK_ESCAPE ) < 0 ){
				break;
			}
		}
	}
	catch ( std::exception& ex ){
		std::cout << ex.what() << std::endl;
	}
}