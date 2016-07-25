//
// Created by garrus on 17.07.16.
//

#include "StepsController.h"
#include "FootTargetFunctor.h"

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
        steps_params.RotY = -35;
        steps_params.NormalSearchRadius = 0.05f;
        steps_params.FootX = 0.40f;
        steps_params.FootY = 0.20f;
        steps_params.SearchX = 0.70f;
        steps_params.SearchY = 0.40;
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
        viewer->removePointCloud(point_cloud_name);
        viewer->addPointCloud(cloud, point_cloud_name.c_str());
    }


    /**
     * Запрос на проверку области на пригодность шага
     */
    StepControllerResponse StepsController::CalculateStep(StepControllerRequest request)
    {

        float z = 0;
        float step = steps_params.DownsampleLeafSize;

        /* Сколько аллокаций тут делается:
         * 1. BoxFilter - выделяется память под новое облако
         * 2. CalculateNormals - под нормали
         * 3. MakeOrganized - под организованное облако и нормали
         * 4. calculate_normal_angles - под массив углов нормалей
         */

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
        CloudTransforms::MakeOrganizedCloud(cropped_cloud, normals,
                                            step, organized, organized_normals,
                                            steps_params.SearchX, steps_params.SearchY,
                                            request.StepX, request.StepY);

        //Расчет углов нормалей к вертикали
        boost::shared_ptr<float[]> normal_angles;
        calculate_noraml_angls(organized_normals, normal_angles);

        //Раскрашиваем точки по углу нормале
        color_cloud_normals(organized, normal_angles);


        //Находим оптимальную точку наступания
        FootTargetFunctor target_func(normal_angles, organized, steps_params, step);
        //target_func(-0.15,0);


        //Пределы поиска (ступня не должна выходить за область поиска)
        float x_max = steps_params.SearchX/2 - steps_params.FootX/2;
        float x_min = -x_max;
        float y_max = steps_params.SearchY/2 - steps_params.FootY/2;
        float y_min = -y_max;

        //Проходим по всем доступным точкам, вычисляем занчение функции
        //и сохраняем в файл в формате X Y Z. Потом можно открыть при помощи
        // GNUPLOT.
        float search_step = 0.01;
        std::ofstream fout;
        fout.open("/home/garrus/plots/plot.txt", fstream::out);
        for(float x = x_min; x<=x_max; x+=search_step)
        {
            for(float y = y_min; y<=y_max; y+=search_step)
            {
                float value = target_func(x,y);
                fout<<x<<" "<<y<<" "<<value<<std::endl;

            }
        }

        fout<<std::fflush;
        fout.close();

        //Убираем NaN, чтобы не глючил визуализатор
        Utils::ReplaceNaNs<pcl::PointXYZRGB>(organized, 0);

        //Визуализация
        viewer->removePointCloud("normals");
        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(organized, organized_normals, 1, 0.015, "normals", 0);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0, "normals");


        viewer->removePointCloud("organized");
        viewer->addPointCloud(organized, "organized");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "organized");

        //Рисуем уровень среднего
        //draw_cube(request.StepX, request.StepY, av_height, steps_params.SearchX, steps_params.SearchY, height_deviation,0,0,1, "average_plane");


        StepControllerResponse response;
        return response;
    }

    //Расчитывает углы между нормалями и вертикалью
    void StepsController::calculate_noraml_angls(pcl::PointCloud<pcl::Normal>::Ptr normals,
                                boost::shared_ptr<float[]> & angles)
    {
        //TODO: корректно ли работает выделение массива? И будет ли он потом сам удалятс?
        //TODO: заменить на std::vector
        angles = boost::make_shared<float[]>(normals->size());

        for(int i = 0; i<normals->size(); i++)
        {
            pcl::Normal normal = normals->at(i);
            float angle = Math::RadToDeg(acos(Math::AngleBetweenLines(normal, Math::Vertical())));

            angles[i]=angle;
        }
    }

    //Задает точкам облака цвета в зависимости от откланения от вертикали
    void StepsController::color_cloud_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                              boost::shared_ptr<float[]> normal_angles)
    {
        for(int i = 0; i<cloud->size(); i++)
        {
            float angle = normal_angles[i];

            double max = 20;
            uint8_t green, red;

            if(fabs(angle)>=max)
                green=0;
            else
                green = (max - fabs(angle)) / max * 255;

            red = 255 - green;

            cloud->at(i).r = red;
            cloud->at(i).g = green;
        }
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