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


    StepsController::StepsController()
    {
        steps_params.DownsampleLeafSize = 0.01f;
        steps_params.ShiftX = 0;
        steps_params.ShiftY = 0;
        steps_params.ShiftZ = 1.5;
        steps_params.RotX = 0;
        steps_params.RotY = -37;
        steps_params.NormalSearchRadius = 0.05f;
        steps_params.FootX = 0.40f;
        steps_params.FootY = 0.20f;
        steps_params.SearchX = 0.70f;
        steps_params.SearchY = 0.20;
        steps_params.SearchZ=1;


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
        pointCloud2 = cloud_transforms.DownsampleCloud(pointCloud2, steps_params.DownsampleLeafSize);

        //Преобразование в pcl::PointCloud<T>
        pcl::PointCloud<POINT_TYPE>::Ptr cloud(new pcl::PointCloud<POINT_TYPE>);
        pcl::fromPCLPointCloud2(*pointCloud2, *cloud);

        //Поворот
        cloud = cloud_transforms.TransformCloud(cloud,
                                                pcl::PointXYZ(steps_params.RotX, steps_params.RotY, steps_params.RotZ),
                                                pcl::PointXYZ(steps_params.ShiftX, steps_params.ShiftY, steps_params.ShiftZ));

        current_cloud = cloud;


        //Обновляем облако в визуализаторе
        //viewer->removePointCloud(point_cloud_name);
        //viewer->addPointCloud(cloud, point_cloud_name.c_str());
    }


    /**
     * Запрос на проверку области на пригодность шага
     */
    StepControllerResponse StepsController::CalculateStep(StepControllerRequest request)
    {

        float z = 0;

        draw_cube(request.StepX, request.StepY, z, steps_params.SearchX, steps_params.SearchY, steps_params.SearchZ, 1,0,0, search_cube_name);
        draw_cube(request.StepX, request.StepY, z, steps_params.FootX, steps_params.FootY, 0.1, 0,1,0, foot_cube_name);

        //Обрезаем облако, чтобы было только область, в которой мы осуществляем поиск
        pcl::PointCloud<POINT_TYPE>::Ptr cropped_cloud = cloud_transforms.BoxFilter(current_cloud, request.StepX,
                                                                                    request.StepY, z,
                                                                                    steps_params.SearchX, steps_params.SearchY, steps_params.SearchZ);

        //Расчет нормалей
        pcl::PointCloud<pcl::Normal>::Ptr normals = cloud_transforms.CalulateNormals(cropped_cloud, steps_params.NormalSearchRadius);

        //Получаем организованное облако и организованные нормали
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr organized;
        pcl::PointCloud<pcl::Normal>::Ptr organized_normals;
        CloudTransforms::MakeOrganizedCloud(cropped_cloud, normals, steps_params.DownsampleLeafSize , organized, organized_normals);


        //viewer->removePointCloud("normals");
        //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(organized, organized_normals, 1, 0.015, "normals", 0);
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0, "normals");


        const float nan = std::numeric_limits<float>::quiet_NaN();
        pcl::PointXYZRGB null_point;
        null_point.x = nan;
        null_point.y = nan;
        null_point.z = nan;

        //Создаем облако нужного размера
        /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr organized =  boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(100, 100, null_point);


        for(int i = 0; i<100; i++)
        {
            for(int j = 0; j<100; j++)
            {
                organized->at(i,j).x=i/100.0f;
                organized->at(i,j).y=j/100.0f;
                organized->at(i,j).z=0;

                organized->at(i,j).r = 255;

            }
        }*/



        viewer->removePointCloud("organized");
        viewer->addPointCloud(organized, "organized");
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0, "organized");*/

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