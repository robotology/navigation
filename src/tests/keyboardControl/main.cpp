/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/MobileBaseVelocity.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <SDL.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

int main(int argc, char* argv[])
 {
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("check Yarp network.\n");
        return -1;
    }

    yarp::os::Port outputport;
    outputport.open("/keyboardControl:o");

    SDL_Event event;
    int quit = 0;

    // Initialise SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        yError("Could not initialise SDL: %s\n", SDL_GetError());
        return -1;
    }

    auto window = SDL_CreateWindow(
        "An SDL2 window",                  // window title
        SDL_WINDOWPOS_UNDEFINED,           // initial x position
        SDL_WINDOWPOS_UNDEFINED,           // initial y position
        500,                               // width, in pixels
        500,                               // height, in pixels
        0                  // flags - see below
    );
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
    SDL_Surface* image = SDL_LoadBMP("controls.bmp");
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, image);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);

    bool pressed_left = false;
    bool pressed_up = false;
    bool pressed_down = false;
    bool pressed_right = false;
    bool pressed_turn_left = false;
    bool pressed_turn_right = false;
    double max_vel_lin =    1; //ms/s
    double max_vel_theta = 10; //deg/s
    yarp::dev::MobileBaseVelocity output_data;

    // Loop until an SDL_QUIT event is found
    while (!quit)
    {
        yarp::os::Time::delay(0.010);
        SDL_PollEvent(&event);

        switch (event.type)
        {
        case SDL_QUIT:
            quit = true;
            break;
        case SDL_KEYDOWN:
            switch (event.key.keysym.sym)
            {
            case SDLK_a: pressed_left=true; break;
            case SDLK_w: pressed_up = true; break;
            case SDLK_s: pressed_down=true; break;
            case SDLK_d: pressed_right = true; break;
            case SDLK_q: pressed_turn_left = true; break;
            case SDLK_e: pressed_turn_right = true; break;
            case SDLK_i: max_vel_lin+=0.05; yDebug() << "linear velocity:" << max_vel_lin << "m/s"; break;
            case SDLK_k: if (max_vel_lin>0) {max_vel_lin -= 0.05;} yDebug() << "Linear velocity:" << max_vel_lin<< "m/s"; break;
            case SDLK_o: max_vel_theta += 5; yDebug() << "angular velocity:" << max_vel_theta << "deg/s"; break;
            case SDLK_l: if (max_vel_theta > 0) {max_vel_theta -= 5;} yDebug() << "Angular velocity:" << max_vel_theta << "deg/s"; break;
            }
            break;
        case SDL_KEYUP:
            switch (event.key.keysym.sym)
            {
            case SDLK_a: pressed_left = false; break;
            case SDLK_w: pressed_up = false; break;
            case SDLK_s: pressed_down = false; break;
            case SDLK_d: pressed_right = false; break;
            case SDLK_q: pressed_turn_left = false; break;
            case SDLK_e: pressed_turn_right = false; break;

            }
        }

        //output_data.clear();
        if (pressed_left)            { output_data.vel_y = max_vel_lin; } 
        else if (pressed_right)      { output_data.vel_y = - max_vel_lin; }
        else                         { output_data.vel_y = 0; }

        if (pressed_up)              { output_data.vel_x = max_vel_lin; }
        else if (pressed_down)       { output_data.vel_x = - max_vel_lin; }
        else                         { output_data.vel_x = 0; }

        if (pressed_turn_left)       { output_data.vel_theta = max_vel_theta; }
        else if (pressed_turn_right) { output_data.vel_theta = - max_vel_theta;  }
        else                         { output_data.vel_theta = 0; }
        
        outputport.write(output_data);

    }

    // Clean up
    SDL_Quit();
    return 0;
}
