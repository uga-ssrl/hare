#include "utility.h"

float euclid(const float2& a, const float2& b){
  return sqrtf(((a.x-b.x)*(a.x-b.x))+((a.y-b.y)*(a.y-b.y)));
}
