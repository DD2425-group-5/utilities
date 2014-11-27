#include <ros/ros.h>
#include <limits>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Polygon.h>
#include <mapping_msgs/Line.h>
#include "mathutil/mathutil.hpp"

namespace MapUtil {

    const uint8_t OCCUPIED = 100;
    const uint8_t UNOCCUPIED = 0;
    const uint8_t UNKNOWN = 255;

    /**
     * Represents a cell in the occupancy grid. Since the grid is discrete, its
     * x,y position is an int.
     */
    struct Cell {
        Cell(int _x, int _y) : x(_x), y(_y){};
        const int x, y;
    };

    mapping_msgs::Line alignLineToAxis(const mapping_msgs::Line& line);
    void alignLineToAxisInPlace(mapping_msgs::Line& line);
    //std::vector<Cell> cellsOnLine(const nav_msgs::MapMetaData& info, mapping_msgs::Line l);
    std::vector<int> indicesOnAlignedLine(const nav_msgs::MapMetaData& info, const mapping_msgs::Line& l);
    std::vector<int> indicesInAlignedRectangle(const nav_msgs::MapMetaData& info, const geometry_msgs::Polygon& poly);
    
    Cell polygonCentre(const nav_msgs::MapMetaData& info, geometry_msgs::Polygon& poly);
    geometry_msgs::Polygon cellAsPolygon(const nav_msgs::MapMetaData& info, Cell c);
    int getCellIndex(const nav_msgs::MapMetaData& info, const Cell c);
    Cell getCellAtIndex(const nav_msgs::MapMetaData& info, const int index);
    Cell getCellAtPoint(const nav_msgs::MapMetaData& info, const float x, const float y);
    bool indexInBounds(const nav_msgs::MapMetaData& info, const int index);
    bool cellInBounds(const nav_msgs::MapMetaData& info, const Cell& c);
    int getIndexAtPoint(const nav_msgs::MapMetaData& info, const float x, const float y);
}
