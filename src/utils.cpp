#include "utils.h"

double above_battery = 0.03;

std::unordered_map<std::string, double> model_height =
{
        {"assembly_battery_red", above_battery},
        {"assembly_battery_green", above_battery},
        {"assembly_battery_blue", above_battery}
};

