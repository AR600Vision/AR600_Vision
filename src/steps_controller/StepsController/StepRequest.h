/**
 * Запрос и ответ на запрос проверки шага
 *
 * Я не стал тащить в зависимость от автогенерированных файлов ROS
 * в свой класс.
 *
 * Придется синхронить их руками
 */

#ifndef AR600_VISION_STEPREQUEST_H
#define AR600_VISION_STEPREQUEST_H

namespace StepsController
{
    
/**
 * Запрос на проверку возможности сделать шаг
 */
    struct StepControllerRequest {
        float StepX, StepY;
    };

/**
 * Ответ на запрос
 */
    struct StepControllerResponse
    {
        bool CanStep;
        float StepX, StepY, StepZ;

        StepControllerResponse()
        {
            CanStep=false;
        };

        StepControllerResponse(bool canStep, float x, float y, float z)
        {
            CanStep = canStep;
            StepX = x;
            StepY = y;
            StepZ = z;
        }
    };
}

#endif //AR600_VISION_STEPREQUEST_H
