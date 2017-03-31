//
// Created by user on 31.03.17.
//

#include "SimpleDraw.h"

SimpleDraw::SimpleDraw(int width, int height)
{
    SDL_Init(SDL_INIT_VIDEO);
    win = SDL_CreateWindow("Obstacle detection", 100, 100, 500, 500, SDL_WINDOW_SHOWN);
    ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    Clear(Color::White());
}

void SimpleDraw::Clear(Color color)
{
    SetColor(color);
    SDL_RenderClear(ren);
}

void SimpleDraw::DrawLine(Color color, int x1, int y1, int x2, int y2)
{
    SetColor(color);
    SDL_RenderDrawLine(ren, x1, y1, x2, y2);
}

void SimpleDraw::DrawRect(Color color, int x, int y, int width, int height)
{
    SetColor(color);

    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = width;
    rect.h = height;
    SDL_RenderDrawRect(ren, &rect);
}

void SimpleDraw::FillRect(Color color, int x, int y, int width, int height)
{
    SetColor(color);

    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = width;
    rect.h = height;
    SDL_RenderFillRect(ren, &rect);
}

void SimpleDraw::DrawEllipse(Color color, int x, int y, int width, int height)
{
    //ellipseRGBA(ren, x, y, width, height, color.R, color.G, color.B, 255);
}

void SimpleDraw::FillEllipse(Color color, int x, int y, int width, int height)
{
    //filledEllipseRGBA(ren, x, y, width, height, color.R, color.G, color.B, 255);
}


void SimpleDraw::Delay(int millis)
{
    SDL_Delay(millis);
}

void SimpleDraw::SetColor(Color color)
{
    SDL_SetRenderDrawColor(ren, color.R, color.G, color.B, 255);
}


void SimpleDraw::Update()
{
    SDL_RenderPresent(ren);
}


