#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include "kdtree/kdtree.h"

struct DataStr{
	int id;
	double x;
	double y;
	double z;
};

int main() {
	kdtree *tree = kd_create(3);
	
	DataStr *d0 = new DataStr{.id=0, .x=0, .y=0, .z=0};
	kd_insert3(tree, d0->x, d0->y, d0->z, d0);

	DataStr *d1 = new DataStr{.id=1, .x=1, .y=0, .z=0};
	kd_insert3(tree, d1->x, d1->y, d1->z, d1);

	DataStr d2 = {.id=2, .x=0, .y=1, .z=0};
	kd_insert3(tree, d2.x, d2.y, d2.z, &d2);

	DataStr d3 = {.id=3, .x=0, .y=0, .z=1};
	kd_insert3(tree, d3.x, d3.y, d3.z, &d3);

	DataStr d4 = {.id=4, .x=1, .y=1, .z=1};
	kd_insert3(tree, d4.x, d4.y, d4.z, &d4);

	kdres *nearest = kd_nearest3(tree, 1, 1, 0.5);
	DataStr * res = (DataStr*) kd_res_item_data(nearest);

  std::cout << "Nearest: " << res->id << "," << res->x << "," << res->y << "," << res->z << std::endl;
  kd_res_free(nearest);

  double range = 1.5;
	kdres *nearest_range = kd_nearest_range3(tree, 1, 1, 0.5, range);
	int nearest_size = kd_res_size(nearest_range);
	std::cout << "Found: " << nearest_size << " neigbors in a range of " << range << " (m)" << std::endl;
	for (int i = 0; i < nearest_size; ++i) {
		res = (DataStr*) kd_res_item_data(nearest_range);
	  std::cout << "Nearest-range: " << res->id << "," << res->x << "," << res->y << "," << res->z << std::endl;
	  kd_res_next(nearest_range);
	}

  kd_res_free(nearest_range);


	nearest_range = kd_nearest_range3(tree, 1, 1, 0.5, range);
	nearest_size = kd_res_size(nearest_range);
	std::cout << "Found: " << nearest_size << " neigbors in a range of " << range << " (m)" << std::endl;
	for (int i = 0; i < nearest_size; ++i) {
		res = (DataStr*) kd_res_item_data(nearest_range);
	  std::cout << "Nearest-range: " << res->id << "," << res->x << "," << res->y << "," << res->z << std::endl;
	  kd_res_next(nearest_range);
	}

  kd_res_free(nearest_range);


  return 0;
}
