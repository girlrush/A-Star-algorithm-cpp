// #pragma once

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "include/A_star.hpp"

using namespace std;

int main()
{
    Pose departure = {0, 0};
    Pose goal = {9, 9};

    std::pair<int, int> map_size = std::make_pair(10, 10);
    int density = 2;

    AStar a_star(departure, goal, map_size, density);
    a_star.Run();

    a_star.PrintMap();

    return 0;
}
