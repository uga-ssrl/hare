#include "utility.h"
// this will take in odom and transfor TO MAP
int2 hare::transform(float x, float y)
{
      return {(MAP_X/2/4 + this->init_pose.x + x),(MAP_Y/2/4 + this->init_pose.y + y)};
}
