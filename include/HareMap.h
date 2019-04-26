#ifndef HAREMAP_H
#define HAREMAP_H
#include "utility.h"
#include "common_includes.h"

typedef struct map_node{
  int chracteristic;
  bool explored;
  bool traversable;
  int4 walls;
} map_node;
typedef struct pq_node{
  int x;
  int y;
  float h; // heuristic priority
} pq_node;

//full map generated in a rediculous way with a python file
extern std::vector<std::vector<map_node>> fullMap;

#endif /* HAREMAP_H */
