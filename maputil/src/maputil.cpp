#include "maputil.hpp"

namespace MapUtil{

    /**
     * Returns a vector of cells which lie on the given line.
     */
    std::vector<Cell> cellsOnLine(const nav_msgs::MapMetaData& info, const mapping_msgs::Line& l){
        
    }

    /**
     * Returns a vector of cells which lie inside the given convex polygon
     */
    std::vector<Cell> cellsInPolygon(const nav_msgs::MapMetaData& info, const geometry_msgs::Polygon& poly){
    }

    /**
     * Flood fill the inside of a convex polygon and return the cells inside it.
     */
    std::vector<Cell> floodFillPolygon(const nav_msgs::MapMetaData& info, const geometry_msgs::Polygon& poly){
        
    }

    /**
     * Compute the centre point of a convex polygon - just take the average of
     * points in the vertices of the polygon. This is valid because of convexity.
     */
    Cell polygonCentre(const nav_msgs::MapMetaData& info, geometry_msgs::Polygon& poly){
        float x = 0;
        float y = 0;
        
        int vertices = (int)poly.points.size();
        for (size_t i = 0; i < poly.points.size(); i++) {
            x += poly.points[i].x;
            y += poly.points[i].y;
        }
        
        x /= vertices;
        y /= vertices;

        return getCellAtPoint(info, x, y);
    }

    /**
     * Convert a cell to a polygon. Should use to check intersection between line and cell
     */
    geometry_msgs::Polygon cellAsPolygon(const nav_msgs::MapMetaData& info, Cell& c){
        geometry_msgs::Polygon p;
        
        // Cell 0,0 has bottom left coordinates (0,0) and top right coordinates
        // (info.resolution,info.resolution). Thus, any other cell has bottom left
        // coordinates corresponding to (c.x*res,c.y*res)
        
        // store the bottom left - each point can be constructed from it by
        // adding the resolution to none, one, or both points
        float blx = c.x * info.resolution;
        float bly = c.y * info.resolution;
        geometry_msgs::Point32 botLeft;
        botLeft.x = blx;
        botLeft.y = bly;
        p.points.push_back(botLeft);
        
        geometry_msgs::Point32 topLeft;
        topLeft.x = blx;
        topLeft.y = bly + info.resolution;
        p.points.push_back(topLeft);

        geometry_msgs::Point32 topRight;
        topRight.x = blx + info.resolution;
        topRight.y = bly + info.resolution;
        p.points.push_back(topRight);

        geometry_msgs::Point32 botRight;
        botRight.x = blx + info.resolution;
        botRight.y = bly;
        p.points.push_back(botRight);
        
        return p;
    }



    /**
     * Returns true if the given cell is intersected by the given line.
     */
    bool cellIntersectsLine(const nav_msgs::MapMetaData& info, const mapping_msgs::Line& l, const Cell&){
        
    }

    /**
     * Get the index of the given cell in the data array corresponding to the
     * given metadata. If the cell is out of bounds of the grid, return -1.
     */
    int getCellIndex(const nav_msgs::MapMetaData& info, const Cell& c){
        if (!cellInBounds(info, c)){
            return -1;
        }
        return c.x * info.width + c.y;
    }

    /**
     * Get the cell which corresponds to the given index. If the cell is out of
     * bounds of the grid, return a cell containing (-1,-1)
     */
    Cell getCellAtIndex(const nav_msgs::MapMetaData& info, const int index){
        if (!indexInBounds(info, index)){
            return Cell(-1,-1);
        }
        
        // grid data is in row major order - row==width?
        // e.g. 4x5 array - index 19 (last element)
        // rowInd = 19/5 = 3 : make use of int properties
        // colInd = 19%4 = 4
        // but for 20, 20/5 = 4 -> next row
        // 20%4 = 0 -> first element in the row
        int rowInd = index / info.width;
        int colInd = index % info.width;

        return Cell(rowInd, colInd);
    }

    /**
     * Return the cell which contains the point defined by the given x and y.
     */
    Cell getCellAtPoint(const nav_msgs::MapMetaData& info, const float x, const float y){
        return Cell((int)(x / info.resolution), (int)(y / info.resolution));
    }

    /**
     * Returns true if the given index is within the bounds of the map
     * represented by the given metadata
     */
    bool indexInBounds(const nav_msgs::MapMetaData& info, const int index){
        return index > -1 && index < (info.width * info.height - 1);
    }

    /**
     * Returns true if the given cell is within the bounds of the map
     * represented by the given metadata
     */
    bool cellInBounds(const nav_msgs::MapMetaData& info, const Cell& c){
        bool xInRange = (c.x > -1 && c.x <= info.width);
        bool yInRange = (c.y > -1 && c.y <= info.height);
        
        return xInRange && yInRange;
    }
    
}
