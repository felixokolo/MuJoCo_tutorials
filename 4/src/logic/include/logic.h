#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include <mujoco/mujoco.h>

#define CYLINDER_SENSOR_DISTANCE 0.1
#define CYLINDER_MID_BELT_DISTANCE 0.125
#define CONVEYOR_SPEED 3 * M_PI

/**
 * Describes the different states of a conveyor light switches
 */
typedef enum LIGHT
{
    LIGHT_NONE = -1,    // No Light switch Active
    LIGHT_LEFT,         // Left Switch active
    LIGHT_RIGHT         // Right Switch active
} LIGHT;

/**
 * Description of sensordata array values
 */
enum SENSORS_ID
{
    // Left Conveyor Belt sensors
    VELOCIMETER_X_LEFT = 0,     // Velocimeter X-Value
    VELOCIMETER_Y_LEFT,         // Velocimeter Y-Value
    VELOCIMETER_Z_LEFT,         // Velocimeter Z-Value
    BELT_TOUCH_LEFT,            // Belt Touch sensor value
    LIGHT_TOUCH_RIGHT_LEFT,     // Light Touch sensor Right value
    LIGHT_TOUCH_LEFT_LEFT,      // Light Touch sensor Left value
    PNEUMATIC_RETRACT_X_LEFT,   // Pneumatic cylinder normal sensor retract X value
    PNEUMATIC_RETRACT_Y_LEFT,   // Pneumatic cylinder normal sensor retract Y value
    PNEUMATIC_RETRACT_Z_LEFT,   // Pneumatic cylinder normal sensor retract Z value
    PNEUMATIC_EXTEND_X_LEFT,    // Pneumatic cylinder normal sensor extend X value
    PNEUMATIC_EXTEND_Y_LEFT,    // Pneumatic cylinder normal sensor extend Y value
    PNEUMATIC_EXTEND_Z_LEFT,    // Pneumatic cylinder normal sensor extend Z value

    // Up Conveyor Belt Sensors
    VELOCIMETER_X_UP,           // Velocimeter X-Value
    VELOCIMETER_Y_UP,           // Velocimeter Y-Value
    VELOCIMETER_Z_UP,           // Velocimeter Z-Value
    BELT_TOUCH_UP,              // Belt Touch sensor value
    LIGHT_TOUCH_RIGHT_UP,       // Light Touch sensor Right value
    LIGHT_TOUCH_LEFT_UP,        // Light Touch sensor Left value

    // Right Conveyor Belt Sensors
    VELOCIMETER_X_RIGHT,        // Velocimeter X-Value
    VELOCIMETER_Y_RIGHT,        // Velocimeter Y-Value
    VELOCIMETER_Z_RIGHT,        // Velocimeter Z-Value
    BELT_TOUCH_RIGHT,           // Belt Touch sensor value
    LIGHT_TOUCH_RIGHT_RIGHT,    // Light Touch sensor Right value
    LIGHT_TOUCH_LEFT_RIGHT,     // Light Touch sensor Left value
    PNEUMATIC_RETRACT_X_RIGHT,  // Pneumatic cylinder normal sensor retract X value
    PNEUMATIC_RETRACT_Y_RIGHT,  // Pneumatic cylinder normal sensor retract Y value
    PNEUMATIC_RETRACT_Z_RIGHT,  // Pneumatic cylinder normal sensor retract Z value
    PNEUMATIC_EXTEND_X_RIGHT,   // Pneumatic cylinder normal sensor extend X value
    PNEUMATIC_EXTEND_Y_RIGHT,   // Pneumatic cylinder normal sensor extend Y value
    PNEUMATIC_EXTEND_Z_RIGHT,   // Pneumatic cylinder normal sensor extend Z value

    // Down Conveyor Belt Sensors
    VELOCIMETER_X_DOWN,             // Velocimeter X-Value
    VELOCIMETER_Y_DOWN,             // Velocimeter Y-Value
    VELOCIMETER_Z_DOWN,             // Velocimeter Z-Value
    BELT_TOUCH_DOWN,                // Belt Touch sensor value
    LIGHT_TOUCH_RIGHT_DOWN,         // Light Touch sensor Right value
    LIGHT_TOUCH_LEFT_DOWN           // Light Touch sensor Left value

};

/**
 * Describes the start address for sensordata values for each conveyor belt.
 */
enum BELT
{
    NONE = -1,
    LEFT = VELOCIMETER_X_LEFT,      // LEFT Conveyor sensordata start address
    UP = VELOCIMETER_X_UP,          // UP Conveyor sensordata start address
    RIGHT = VELOCIMETER_X_RIGHT,    // RIGHT Conveyor sensordata start address
    DOWN = VELOCIMETER_X_DOWN,      // DOWN Conveyor sensordata start address

    BELT_NUM
} typedef BELT;

/**
 * Describes relative position of each sensor values for each conveyor belt
 * for easy access in combination with the start address BELT.
 */
enum SENSORS
{
    VELOCIMETER_X = 0,              // Velocimeter X-Value
    VELOCIMETER_Y,                  // Velocimeter Y-Value
    VELOCIMETER_Z,                  // Velocimeter Z-Value
    BELT_TOUCH,                     // Belt Touch sensor value
    LIGHT_TOUCH_RIGHT,              // Light Touch sensor Right value
    LIGHT_TOUCH_LEFT,               // Light Touch sensor Left value
    PNEUMATIC_RETRACT_X,            // Pneumatic cylinder normal sensor retract X value
    PNEUMATIC_RETRACT_Y,            // Pneumatic cylinder normal sensor retract Y value
    PNEUMATIC_RETRACT_Z,            // Pneumatic cylinder normal sensor retract Z value
    PNEUMATIC_EXTEND_X,             // Pneumatic cylinder normal sensor extend X value
    PNEUMATIC_EXTEND_Y,             // Pneumatic cylinder normal sensor extend Y value
    PNEUMATIC_EXTEND_Z,             // Pneumatic cylinder normal sensor extend Z value

    CONV_SENSOR_NUM
};

/**
 * Describes the each actuator in the ctrl array.
 */
enum ACTUATORS
{
    VELOCITY_LEFT = 0,              // Roller velocity actuator for left conveyor belt.
    PNEUMATIC_LEFT,                 // Pneumatic velocity actuator for left conveyor belt.
    VELOCITY_UP,                    // Roller velocity actuator for up conveyor belt.
    VELOCITY_RIGHT,                 // Roller velocity actuator for right conveyor belt.
    PNEUMATIC_RIGHT,                // Pneumatic velocity actuator for right conveyor belt.
    VELOCITY_DOWN,                  // Roller velocity actuator for down conveyor belt.
    VELOCITY_BOX                    // Box velocity actuator.
};

/**
 * Describes XYZ Cartesian directions.
 */
enum VELOCITY_DIR
{
    DIR_X = 0,  // X-Direction
    DIR_Y,      // Y-Direction
    DIR_Z       // Z-Direction
};

/**
 * Describes direction for box velocity actuator gear.
 */
enum BOX_DIR
{ 
    DIR_NEGATIVE = -1,  // Negative direction
    DIR_NONE,           // No direction
    DIR_POSITIVE        // Positive direction
    
};

/**
 * Describes direction for pneumatic cylinder.
 */
typedef enum CYLINDER_DIR
{
    EXTEND,         // Extend cylinder.
    RETRACT         // Retract cylinder.
} CYLINDER_DIR;

/**
 * Returns the starting address of sensordata for a specified conveyor belt.
 * @param m - MuJoCo model variable.
 * @param d - MUJoCo data variable.
 * @param id - ID of conveyor belt.
 * @return Pointer to sensordata array
 */
double *get_conv_sensor_addr(const mjModel* m, mjData* d, BELT id);

/**
 * Checks if an object touches a conveyor belt
 * @param m - MuJoCo model variable.
 * @param d - MUJoCo data variable.
 * @param id - ID of conveyor belt to check.
 * @return True if object on conveyor else False.
 */
bool check_conv_touch(const mjModel* m, mjData* d, BELT id);

/**
 * Changes the velocity actuator direction of the box based on the
 * a reference conveyor.
 * @param m - MuJoCo model variable.
 * @param d - MUJoCo data variable.
 * @param id - ID of ref conveyor belt.
 */
void change_box_vel(const mjModel* m, mjData* d, BELT id);

/**
 * Gets the speed of a conveyor belt from velocimeter sensor.
 * @param m - MuJoCo model variable.
 * @param d - MUJoCo data variable.
 * @param id - ID of conveyor belt.
 * @return Value of velocimeter sensor
 */
double get_conv_speed(const mjModel* m, mjData* d, BELT id);

/**
 * Set box velocity.
 * @param vel Velocity value to set.
 */
void set_box_speed(const mjModel* m, mjData* d, double speed);

/**
 * Checks the state of each light switch on a conveyor belt.
 * @param m - MuJoCo model variable.
 * @param d - MUJoCo data variable.
 * @param id - ID of conveyor belt.
 * @return A LIGHT state decription of which light is active.
 */
LIGHT check_lights(const mjModel* m, mjData* d, BELT id);

/**
 * Checks ifa conveyor cylinder is extended.
 * @param m - MuJoCo model variable.
 * @param d - MUJoCo data variable.
 * @param id - ID of conveyor belt.
 * @return True if extended else false.
 */
bool cylinder_extended(const mjModel* m, mjData* d, BELT id);

/**
 * Checks ifa conveyor cylinder is retracted.
 * @param m - MuJoCo model variable.
 * @param d - MUJoCo data variable.
 * @param id - ID of conveyor belt.
 * @return True if retracted else false.
 */
bool cylinder_retracted(const mjModel* m, mjData* d, BELT id);

/**
 * Main conrol callback logic for simulation.
 * @param m - MuJoCo model variable.
 * @param d - MUJoCo data variable.
 */
void controller(const mjModel* m, mjData* d);
