#include "maputil.hpp"

namespace MapUtil {


    /**
     * Aligns a line to one of the axes - this is automatically determined based
     * on the start and end points of the line - whichever pair of coordinates
     * is closer is set to the average. e.g. if start.x and end.x are closer to
     * each other than start.y and end.y, the x coordinates of the returned line
     * are set to (start.x+end.x)/2, the y coordinates remain the same.
     */
    mapping_msgs::Line alignLineToAxis(const mapping_msgs::Line& line){
        geometry_msgs::Point newStart;
        geometry_msgs::Point newEnd;
        
        if (std::fabs(line.start.x - line.end.x) < std::fabs(line.start.y - line.end.y)){
            float newX = (line.start.x + line.end.x)/2;
            newStart.x = newX;
            newStart.y = line.start.y;
            newEnd.x = newX;
            newEnd.y = line.end.y;
            
        } else {
            float newY = (line.start.y + line.end.y)/2;
            newStart.x = line.start.x;
            newStart.y = newY;
            newEnd.x = line.end.x;
            newEnd.y = newY;
        }
        
        mapping_msgs::Line newLine;
        newLine.start = newStart;
        newLine.end = newEnd;
        return newLine;
    }

    /**
     * Aligns a line to one of the axes - this is automatically determined based
     * on the start and end points of the line - whichever pair of coordinates
     * is closer is set to the average. e.g. if start.x and end.x are closer to
     * each other than start.y and end.y, the x coordinates of the line
     * are set to (start.x+end.x)/2, the y coordinates remain the same.
     * 
     * The line given is modified in place.
     */
    void alignLineToAxisInPlace(mapping_msgs::Line& line){
        geometry_msgs::Point newStart;
        geometry_msgs::Point newEnd;
        
        if (std::fabs(line.start.x - line.end.x) < std::fabs(line.start.y - line.end.y)){
            float newX = (line.start.x + line.end.x)/2;
            newStart.x = newX;
            newStart.y = line.start.y;
            newEnd.x = newX;
            newEnd.y = line.end.y;
        } else {
            float newY = (line.start.y + line.end.y)/2;
            newStart.x = line.start.x;
            newStart.y = newY;
            newEnd.x = line.end.x;
            newEnd.y = newY;
        }
        
        line.start = newStart;
        line.end = newEnd;
    }

    /**
     * Compute the indices of cells on a line which is aligned with either the x
     * or y axis.
     */
    std::vector<int> indicesOnAlignedLine(const nav_msgs::MapMetaData& info,
                                          const mapping_msgs::Line& l){
        ROS_DEBUG_STREAM("Aligned line: " << l);
        std::vector<int> indices;
        if (MathUtil::approxEqual(l.start.y, l.end.y, 0.001)){
            // if the line is along the x axis, then we look at rows in the
            // occupancy grid - maximum length of the line is the number of
            // cells in the width of the grid
            float startX, startY, endX, endY;
            if (l.start.x < l.end.x){
                ROS_DEBUG("start.x < end.x");
                startX = l.start.x;
                startY = l.start.y;
                endX = l.end.x;
                endY = l.end.y;
            } else {
                startX = l.end.x;
                startY = l.end.y;
                endX = l.start.x;
                endY = l.start.y;
            }
            
            int startInd = getIndexAtPoint(info, startX, startY);
            int endInd = getIndexAtPoint(info, endX, endY);
            ROS_DEBUG("Start index: %d", startInd);
            ROS_DEBUG("End index: %d", endInd);
            for (int i = startInd; i < endInd; i++) {
                indices.push_back(i);
            }
        } else {
            // if the line is along the y axis, then we look at columns in the
            // occupancy grid - maximum length of the line is the number of
            // cells in the height of the grid
            float startX, startY, endX, endY;
            if (l.start.y < l.end.y){
                startX = l.start.x;
                startY = l.start.y;
                endX = l.end.x;
                endY = l.end.y;
            } else {
                startX = l.end.x;
                startY = l.end.y;
                endX = l.start.x;
                endY = l.start.y;
            }
            int startInd = getIndexAtPoint(info, startX, startY);
            // number of cells between the start and end points
            int rowsBetweenPoints = (int)(std::fabs(startY - endY)/info.resolution);
            ROS_DEBUG("Start index: %d", startInd);
            ROS_DEBUG("Rows between %f and %f: %d", startY, endY, rowsBetweenPoints);
            for (int i = 0; i < rowsBetweenPoints; i++) {
                // each consecutive cell is at the same index in the row of the
                // grid, just need to add the width of the grid to each
                // consecutive point.
                indices.push_back(startInd + i * info.width);
                ROS_DEBUG("Pushing index %d", startInd + i * info.width);
            }
        }

        ROS_DEBUG("Indices size: %d", (int)indices.size());
        return indices;
    }

    /**
     * Finds the cells which lie inside the given rectangle, which is aligned to
     * the x and y axes.
     */
    std::vector<int> indicesInAlignedRectangle(const nav_msgs::MapMetaData& info,
                                               const geometry_msgs::Polygon& poly){
        std::vector<int> indices;
        // find the min and max values of the rectangle
        float minX = std::numeric_limits<float>::max();
        float maxX = -std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        float maxY = -std::numeric_limits<float>::max();
        for (size_t i = 0; i < poly.points.size(); i++) {
            geometry_msgs::Point32 pt = poly.points[i];
            if (pt.x < minX){
                minX = pt.x;
            }
            if (pt.x > maxX){
                maxX = pt.x;
            }
            if (pt.y < minY){
                minY = pt.y;
            }
            if (pt.y > maxY){
                maxY = pt.y;
            }
        }

        // compute the width and height of the rectangle in cells from the min
        // and max values
        int width = std::fabs(maxX - minX) / info.resolution;
        int height = std::fabs(maxY - minY) / info.resolution;
        // find the index of the point at the bottom left corner
        int lowestInd = getIndexAtPoint(info, minX, minY);
        for (int i = 0; i <= height; i++) {
            for (int j = 0; j <= width; j++) {
                // i*info.width gives the offset from the bottom left point to
                // subsequent points along the left edge of the rectangle.
                indices.push_back(lowestInd + i * info.width + j);
            }   
        }
        return indices;
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
     * Get the index of the given cell in the data array corresponding to the
     * given metadata. If the cell is out of bounds of the grid, return -1.
     */
    int getCellIndex(const nav_msgs::MapMetaData& info, const Cell& c){
        if (!cellInBounds(info, c)){
            return -1;
        }
        return c.y * info.width + c.x;
    }

    /**
     * Get the cell which corresponds to the given index. If the cell is out of
     * bounds of the grid, return a cell containing (-1,-1)
     */
    Cell getCellAtIndex(const nav_msgs::MapMetaData& info, int index){
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
        ROS_DEBUG("Row, col of index %d: %d, %d", index, rowInd, colInd);

        return Cell(colInd, rowInd);
    }

    int getIndexAtPoint(const nav_msgs::MapMetaData& info, float x, float y){
        int xCoord = x / info.resolution;
        int yCoord = y / info.resolution;
        // zero based indexing, need to modify nonzero values of y and x
        yCoord = yCoord == 0 ? yCoord : yCoord - 1;
        xCoord = xCoord == 0 ? xCoord : xCoord - 1;
        
        int ind = yCoord * info.width + xCoord;

        ROS_DEBUG("Index at point %f, %f: %d", x, y, ind);
        return ind;
    }

    /**
     * Return the cell which contains the point defined by the given x and y.
     */
    Cell getCellAtPoint(const nav_msgs::MapMetaData& info, float x, float y){
        return Cell((int)(x / info.resolution), (int)(y / info.resolution));
    }

    /**
     * Returns true if the given index is within the bounds of the map
     * represented by the given metadata
     */
    bool indexInBounds(const nav_msgs::MapMetaData& info, int index){
        return index > -1 && index < (info.width * info.height - 1);
    }

    /**
     * Returns true if the given cell is within the bounds of the map
     * represented by the given metadata
     */
    bool cellInBounds(const nav_msgs::MapMetaData& info, const Cell& c){
        bool xInRange = (c.x > -1 && c.x < info.width);
        bool yInRange = (c.y > -1 && c.y < info.height);
        
        return xInRange && yInRange;
    }
    
}
