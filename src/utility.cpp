#include "utility.h"

float euclid(const float2& a, const float2& b){
  return sqrtf(((a.x-b.x)*(a.x-b.x))+((a.y-b.y)*(a.y-b.y)));
}
int manhattan(const int2& a, const int2& b){
  return abs(a.x-b.x) + abs(a.y-b.y);
}
