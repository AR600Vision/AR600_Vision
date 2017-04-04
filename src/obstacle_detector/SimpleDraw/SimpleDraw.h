//
// Created by user on 31.03.17.
//

#ifndef AR600_VISION_SIMPLEDRAW_H
#define AR600_VISION_SIMPLEDRAW_H

#include <SDL2/SDL.h>
//#include <SDL2/SDL2_gfxPrimitives.h>

#include <thread>

struct Color
{
public:
    int R, G, B;
    Color(){}
    Color(int r, int g, int b)
    {
        R=r;
        G=g;
        B=b;
    }

    static Color White()
    {
        return Color(255,255,255);
    }

    static Color Black()
    {
        return Color(0, 0, 0);
    }
};

class SimpleDraw
{
public:
    SimpleDraw(int width, int height);

    void Clear(Color color);
    void DrawLine(Color color, int x1, int y1, int x2, int y2);
    void DrawRect(Color color, int x, int y, int width, int height);
    void FillRect(Color color, int x, int y, int width, int height);
    void DrawEllipse(Color color, int x, int y, int width, int height);
    void FillEllipse(Color color, int x, int y, int width, int height);

    void SetColor(Color color);
    void Update();

    //Задержка
    void Delay(int millis);

    //Нужно вызывать, чтобы окно не висло
    bool Tick();
private:


    std::thread draw_thread;
    bool _isDraw;

    SDL_Window *win;
    SDL_Renderer *ren;

    void draw();
};



#endif //AR600_VISION_SIMPLEDRAW_H
