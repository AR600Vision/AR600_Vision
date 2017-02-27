//
// Created by garrus on 20.02.17.
//

#ifndef AR600_VISION_NODEMEDIATORBASE_H
#define AR600_VISION_NODEMEDIATORBASE_H

#include <mutex>
#include <exception>
#include <cstring>
#include <functional>

/*!
 * @brief Базовый класс, реализующий взаимодействие с нодой
 *
 * Релизует блокировки, установку флагов.
 * Выборку данных из массива, вызов рос, коллбеки,
 * упаковку данных в массив реализуют потомки.
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
     * @param maxBufferSize размер буфера отпрвки, который будет выделен
     */
    NodeMediatorBase(int maxBufferSize);
    ~NodeMediatorBase();


    /*!
     * @brief Отправляет запрос ноде
     * @param buffer буфер с данными
     * @param count размер входящего массива с параметрами
     */
    void SendRequest(const double* buffer, int count);

    /*!
     * @brief Копирует результат в буфер
     * @param buffer буфер для результата
     * @param maxCount максимальный размер, который можно поместить
     *                 в буфер
     * @return размер данных, скопированных в буфер
     *
     * Если объем копируемых данных больше, чем максимальный
     * размер буфера, будет выброшено исключение std::overflow_error
     */
    int ReadResponse(double* buffer,  int maxCount);


protected:

    /*!
     * Преобразует массив double в нужные параметры
     * и отправляет в топик
     * @param buffer
     * @param count
     */
    virtual void _SendRequest(const double* buffer, int count) = 0;

    /*!
     * @brief Сигнализирует о том, что результат получен
     * @param setter лямбда-функция, заносящая данные
     *               в буффер
     *
     * Обертка над SetData, управляющая
     * синхронизацией и флагами
     */
    void Done(std::function<void(double*, int & count, int maxCount)> setter);

private:
    std::mutex _bufferMutex;        //Мьютекс
    bool _isCalcFinished;

    double* SendBuffer;            //Буфер для отправки по сети
    int DataSize;                  //Текущий размер полезных данных в буфере
    const int BufferMaxSize;       //Максимальный размер буфера
};


#endif //AR600_VISION_NODEMEDIATOR_H
