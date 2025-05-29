#include "logic.h"


double *get_conv_sensor_addr(const mjModel* m, mjData* d, BELT id)
{
  return d->sensordata + id ;
}


bool check_conv_touch(const mjModel* m, mjData* d, BELT id)
{
  double *sensor_addr = get_conv_sensor_addr(m, d, id);
  double touch_val = sensor_addr[BELT_TOUCH];

  return touch_val > 0;
}


void change_box_vel(const mjModel* m, mjData* d, BELT id)
{
  set_box_speed(m, d, 0);
  int8_t gear_X = 0;
  int8_t gear_Y = 0;

  switch (id)
  {
  case LEFT:
    gear_X = DIR_NEGATIVE;
    gear_Y = DIR_NONE;
    break;

  case UP:
    gear_X = DIR_NONE;
    gear_Y = DIR_NEGATIVE;
    break;

  case RIGHT:
    gear_X = DIR_POSITIVE;
    gear_Y = DIR_NONE;
    break;

  case DOWN:
    gear_X = DIR_NONE;
    gear_Y = DIR_POSITIVE;
    break;
  
  default:
    break;
  }

  m->actuator_gear[6 * VELOCITY_BOX + DIR_X] = gear_X;
  m->actuator_gear[6 * VELOCITY_BOX + DIR_Y] = gear_Y;
}


double get_conv_speed(const mjModel* m, mjData* d, BELT id)
{
  double* sensor_addr = get_conv_sensor_addr(m, d, id);
  return sensor_addr[VELOCIMETER_X];
}


void set_box_speed(const mjModel* m, mjData* d, double speed)
{
  d->ctrl[VELOCITY_BOX] = speed;
}


LIGHT check_lights(const mjModel* m, mjData* d, BELT id)
{
  if (get_conv_sensor_addr(m, d, id)[LIGHT_TOUCH_LEFT])
  {
    return LIGHT_LEFT;
  }
  else if (get_conv_sensor_addr(m, d, id)[LIGHT_TOUCH_RIGHT])
  {
    return LIGHT_RIGHT;
  }
  else
    return LIGHT_NONE;

}


bool cylinder_extended(const mjModel* m, mjData* d, BELT id)
{
  double *sensor_addr = get_conv_sensor_addr(m, d, id);
  double sensor_abs_val = sqrt(pow(sensor_addr[PNEUMATIC_EXTEND_X], 2) +
                               pow(sensor_addr[PNEUMATIC_EXTEND_Y], 2) +
                               pow(sensor_addr[PNEUMATIC_EXTEND_Z], 2));
  return !(sensor_abs_val > 0);
}

bool cylinder_retracted(const mjModel* m, mjData* d, BELT id)
{
  double *sensor_addr = get_conv_sensor_addr(m, d, id);
  double sensor_abs_val = sqrt(pow(sensor_addr[PNEUMATIC_RETRACT_X], 2) +
                               pow(sensor_addr[PNEUMATIC_RETRACT_Y], 2) +
                               pow(sensor_addr[PNEUMATIC_RETRACT_Z], 2));
  return (sensor_abs_val > 0);
}


void cylinder_control(const mjModel* m, mjData* d, BELT id, CYLINDER_DIR dir)
{
  double conv_speed = get_conv_speed(m, d, id);
  double cylinder_speed = fabs(conv_speed * (CYLINDER_MID_BELT_DISTANCE / CYLINDER_SENSOR_DISTANCE));

  if (dir == RETRACT)
    cylinder_speed = -cylinder_speed;

  switch (id)
  {
  case LEFT:
      d->ctrl[PNEUMATIC_LEFT] = cylinder_speed;

    break;

  case RIGHT:
    d->ctrl[PNEUMATIC_RIGHT] = cylinder_speed;
    break;
  
  default:
    break;
  }
}

void controller(const mjModel* m, mjData* d)
{

  // Keep rack of the last belt where the object was.
  static BELT last_active_belt = NONE;

  // Set the conveyor speed for each conveyor.
  d->ctrl[VELOCITY_LEFT] = CONVEYOR_SPEED;
  d->ctrl[VELOCITY_RIGHT] = CONVEYOR_SPEED;
  d->ctrl[VELOCITY_UP] = CONVEYOR_SPEED * 2;
  d->ctrl[VELOCITY_DOWN] = CONVEYOR_SPEED * 3;

  for (BELT active_belt = LEFT; active_belt < BELT_NUM; active_belt++)
  {
    switch (active_belt)
    {

    case LEFT:
    case RIGHT:
    case UP:
    case DOWN:

      if (check_conv_touch(m, d, active_belt))
      {
        if (last_active_belt != active_belt)
          change_box_vel(m, d, active_belt);

        if (last_active_belt != active_belt)
        {
          cylinder_control(m, d, last_active_belt, RETRACT);
        }

        double box_speed = get_conv_speed(m, d, active_belt);
        set_box_speed(m, d, -box_speed);

        if (check_lights(m, d, active_belt) == LIGHT_RIGHT)
          cylinder_control(m, d, active_belt, EXTEND);

        last_active_belt = active_belt;
      }
      
      break;
    
    default:
      continue;
      break;
    }
    
    
  }

}

