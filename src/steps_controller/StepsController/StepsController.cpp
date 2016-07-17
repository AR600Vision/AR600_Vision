//
// Created by garrus on 17.07.16.
//

#include "StepsController.h"

//using namespace StepsController;

namespace StepsController {

    const std::string StepsController::point_cloud_name = "point_cloud";
    const std::string StepsController::search_cloud_name = "search_point_cloud";
    const std::string StepsController::search_cube_name = "search_cube";
    const std::string StepsController::foot_cube_name = "foot_cube";


    StepsController::StepsController() {
        viewer = setup_vizualizer();

    }

    //Создает визуализатор
    boost::shared_ptr<pcl::visualization::PCLVisualizer> StepsController::setup_vizualizer() {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
        //viewer->initCameraParameters ();
        viewer->setCameraPosition(0, 0, 0, 1, 0, 0, 0, 0, 1, 0);

        return viewer;
    }

    //////////////////////////////////////// ЛОГИКА ОБРАБОТКИ //////////////////////////////////////////////////////////

    /**
     * Принимает очередное изменения облака точек
     * и обрабатывает его
     */
    //TODO: Проверить на утечки памяти
    void StepsController::UpdateFrame(pcl::PCLPointCloud2::Ptr pointCloud2)
    {
        //Уменьшение размера
        pointCloud2 = cloud_transforms.DownsampleCloud(pointCloud2);

        //Преобразование в pcl::PointCloud<T>
        pcl::PointCloud<POINT_TYPE>::Ptr cloud(new pcl::PointCloud<POINT_TYPE>);
        pcl::fromPCLPointCloud2(*pointCloud2, *cloud);

        //Поворот
        cloud = cloud_transforms.RotateCloud(cloud);

        current_cloud = cloud;



        //Обновляем облако в визуализаторе
        viewer->removePointCloud(point_cloud_name);
        viewer->addPointCloud(cloud, point_cloud_name.c_str());
    }


    /**
     * Запрос на проверку области на пригодность шага
     */
    StepControllerResponse StepsController::CalculateStep(StepControllerRequest request)
    {

        float z = 0;

        draw_cube(request.StepX, request.StepY, z, search_x, search_y, search_z, 1,0,0, search_cube_name);
        draw_cube(request.StepX, request.StepY, z, foot_x, foot_y, 0.1, 0,1,0, foot_cube_name);

        //Обрезаем облако, чтобы было только область, в которой мы осуществляем поиск
        pcl::PointCloud<POINT_TYPE>::Ptr cropped_cloud = cloud_transforms.PassThroughFilter(current_cloud, request.StepX, request.StepY, z, search_x, search_y, search_z);

        //Расчет нормалей
        pcl::PointCloud<pcl::Normal>::Ptr normals = cloud_transforms.CalulateNormals(cropped_cloud);

        viewer->removePointCloud("normals");
        viewer->addPointCloudNormals<POINT_TYPE, pcl::Normal>(cropped_cloud, normals, 1, 0.015, "normals", 0);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0, "normals");
        

        StepControllerResponse response;
        return response;
    }


    ////////////////////////////// ВСПОМОГАТЕЛЬНЫЕ ФУКНЦИИ /////////////////////////////////////////////////////////////

    //Обертки над Viewer

    bool StepsController::wasStopped() const
    {
        return viewer->wasStopped();
    }

    void StepsController::spinOnce(int time, bool force_redraw)
    {
        viewer->spinOnce(time, force_redraw);
    }

    //Обертка для более удобного рисования кубов
    //Задается координатами центра и размерами
    // depth - размер по x
    // width - размер по y
    // height - размер по  z
    void StepsController::draw_cube(float x, float y, float z,
                   float depth, float width, float height,
                   float r, float g, float b,
                   const std::string &id)
    {
        viewer->removeShape(id);

        viewer->addCube(
                x-depth/2,  x+depth/2,
                y-width/2,  y+width/2,
                z-height/2, z+height/2,
                r, g, b, id
        );
    }
}