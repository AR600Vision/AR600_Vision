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
     * @param bufferMaxSize размер ответа
     */
    NodeMediatorBase(int bufferMaxSize);
    ~NodeMediatorBase();


    /*!
     * @brief Отправляет запрос ноде
     * @param buffer буфер с данными
     * @param count размер входящего массива с параметрами
     * @return успешность вызова, false - количество параметров недосточно
     */
    bool SendRequest(const double* buffer, int count);

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
     * Количество требуемых параметров
     * @return
     */
    virtual uint8_t RequestLength()=0;

protected:

    /*!
     * Преобразует массив double в нужные параметры
     * и отправляет в топик
     * @param buffer
     * @param count
     */
    virtual bool _SendRequest(const double* buffer, int count) = 0;

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
    bool _firstCall;

    double* SendBuffer;            //Буфер для отправки по сети
    const int BufferMaxSize;       //Максимальный размер буфера
    int DataSize;                  //Текущий размер полезных данных в буфере
};


#endif //AR600_VISION_NODEMEDIATOR_H
