//
// Created by garrus on 20.02.17.
//

#ifndef AR600_VISION_NODEMEDIATORBASE_H
#define AR600_VISION_NODEMEDIATORBASE_H

#include <mutex>
#include <exception>
#include <cstring>
#include <functional>
#include <ros/ros.h>

/*!
 * Результат отправки запроса
 */
enum SendStatus
{
    DONE,
    PUBLISHED,
    ERROR
};


/*!
 * @brief Базовый класс, реализующий взаимодействие с нодой
 *
 *
 * Разработан, чтобы уменьшить количество аллокаций памяти.
 * Поэтому он работает с буфером отправки фиксировнаного размера -
 * накладывает ограничение на размер передаваемых данных, надо предусмотреть заранее.
 *
 */
class NodeMediatorBase
{
public:

    /*!
     * Конструктор
     * @param bufferMaxSize размер ответа
     */
    NodeMediatorBase(int bufferMaxSize);
    ~NodeMediatorBase();


    /*!
     * @brief Копирует результат в буфер
     * @param buffer буфер для результата. Если параметр NULL, то
     *        будет возвращен требуемый размер памяти (в double), без
     *        самого копирования
     * @param maxCount максимальный размер, который можно поместить
     *        в буфер
     * @return размер данных (в double), скопированных в буфер
     *
     * Если объем копируемых данных больше, чем максимальный
     * размер буфера, будет возвращено -1
     */
    int ReadResponse(double* buffer,  int maxCount);

    /*!
     *
     * @return завершен ли расчет
     */
    bool IsDone();

protected:

    const int BufferMaxSize;       //Максимальный размер буфера
    double* SendBuffer;            //Буфер для отправки по сети

    std::mutex Mutex;              //Мьютекс
    bool isDone;

private:

};


#endif //AR600_VISION_NODEMEDIATOR_H
