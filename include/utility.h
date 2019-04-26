#ifndef UTILITY_H
#define UTILITY_H

#include "common_includes.h"

typedef struct float2{
  float x;
  float y;
} float2;
typedef struct float3{
  float x;
  float y;
  float z;
} float3;
typedef struct float4{
  float x;
  float y;
  float z;
  float w;
} float4;

typedef struct int2{
  int x;
  int y;
} int2;
typedef struct int3{
  int x;
  int y;
  int z;
} int3;
typedef struct int4{
  int x;
  int y;
  int z;
  int w;
} int4;

typedef struct map_node{
  uint8_t obstacle;
  bool explored;
  bool traversable;
  int4 walls;
} map_node;
typedef struct pq_node{
  int x;
  int y;
  float h; // heuristic priority
} pq_node;

#endif /* UTILITY_H */
