#include "Arduino.h"

// low computation low memory moveing average filter to adaptively calculate the balance angle of the cube.
// this makes the cube able to slidely adjust the angle it balances around to be more acuraate 
// and at the same time it can hanly slight differences in weight distribution.

float acum_angle = 0;
long int count = 0;
int max_count = 1000;
float values[1000];

bool value_array_isfull = false;

float calculate_avg_angle(float cube_angle)
{
  float oldest_value = 0;
  if (value_array_isfull)
  {
      oldest_value = values[count];
      values[count] = cube_angle;
  }
  else
  {
      values[count] =  cube_angle;
  }
  
  
  if (!value_array_isfull)
  {
      if (count+1 >= max_count)
      {
          value_array_isfull = true;
      }
  }
  else
  {
      acum_angle -= oldest_value;
  }
  
  
  
  acum_angle += cube_angle;
  
  
  
  count = (count+1) % max_count;
  
  float result = -1;
  if (value_array_isfull)
  {
      result = acum_angle/max_count;
  }
  else
  {
      result = acum_angle/count;
  }
  
  return result;
}
