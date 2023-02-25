#include "map_management/RouteManager.hpp"

using namespace navigator::planning;

RouteManager::RouteManager()
{
}

/**
 * @brief Returns a refined route given current position and the rough route
 *
 * @param rough A rough route, with a minimum of two waypoints
 * @param pos Our current position
 * @return A refined path, with smooth and rough sections, and with the route behind us removed.
 */
LineString RouteManager::getRoute(const LineString rough, const BoostPoint pos)
{
    std::printf("Rough route has %i waypoints\n", rough.size());
}