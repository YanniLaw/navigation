/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_COSTMAP_2D_H_
#define COSTMAP_2D_COSTMAP_2D_H_

#include <vector>
#include <queue>
#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>

namespace costmap_2d
{

/*********************************************************************
 * 对导航包坐标系做说明如下，以防疑惑
 *
 *                       rviz显示视图                                                      逻辑上的地图
 *                                                   ^ X              Y ^
 *      ---------------------------------------------|                  |--------------------------------------------
 *      |                                            |                  |                                           |
 *      |                                            |                  |                                           |
 *      |                                            |                  |                                           |
 *      |                                            |                  |                                           |
 *      |                        ^ X                 |                  |             Y ^                           |
 *      |                        |                   |                  |               |                           |
 *      |                        |                   |  clockwise 90°   |               |                           |
 *      |                        |                   |  ============>   |               |                           |
 *      |                        |                   |                  |               |                           |
 *      |        <---------------|                   |  <============   |               |-------------->            |
 *      |        Y        map/odom frame             | anticlockwise 90°|       map/odom frame         X            |
 *      |                   origin(0, 0)             |                  |        origin(0, 0)                       |
 *      |                                            |                  |                                           |
 *      |                                            |                  |                                           |
 *      |                                            |                  |                                           |
 *      |                                            |                  |                                           |
 *      |                                            |                  |                                           |
 *   <------------------------------------------------                  ------------------------------------------------>
 *   Y                                               costmap frame origin                                               X
 *                                                   (origin_x, origin_y)
 *
 *  全局坐标系和代价图的坐标如上右图（逻辑上的地图）所示
 * （1）全局代价图的全局坐标系是map，对于我们的工程（单个机器人）来说，可以直接和世界坐标系对齐
 * （2）局部代价图的全局坐标系是odom，对于我们的工程（单个机器人）来说，可以直接和世界坐标系对齐
 * （3）map和odom的坐标系原点都位于机器人上电时的起始点
 * （4）全局代价图和局部代价图的机器人坐标系都是base_link，机器人正前方为X轴正方向，左手边为Y轴正方向，
 *      坐标系原点位于机器人旋转中心（注：base_footprint是base_link在地面的投影）
 * （5）map和odom在机器人启动的初始时刻是重合的，随着机器人运动，odom的计算会有误差累计（漂移），所以发布出来的坐标转换也有漂移，导致odom和map产生偏差
 * （6）odom因为是轮子编码器和IMU等积分出来的，坐标不会发生跳跃突变，所以是一个短期稳定的坐标系，可以用于局部操作如避障
 * （7）map坐标系的坐标是通过传感器的信息不断的计算更新而来，比如激光雷达，视觉定位等等，因此能够有效的减少累积误差，所以是一个很有用的长期全局坐标系；
 *      但是也导致每次坐标更新可能会产生跳跃，所以是一个比较差的局部坐标系（不适合用于避障和局部操作）
 * （8）代价图的原点位于地图的左下方，我们认为代价图的原点是(0, 0)，在世界坐标系（map/odom）下的表示是（origin_x, origin_y）
 * （9）代码中的mapToWorld和worldToMap等函数，并不是说在map坐标系和世界坐标系之间转换，而是在costmap坐标系和map/odom坐标系之间转换
 * （10）rviz中显示的地图如上左图所示，costmap、map、odom的坐标系原点都在右下方，可能是rviz基于观察习惯的原因，
 *      将逻辑上的地图逆时针旋转了90度来做显示，使机器人初始正前方朝向向上
*********************************************************************/

// convenient for storing x/y point pairs
struct MapLocation
{
  unsigned int x;
  unsigned int y;
};

/**
 * @class Costmap2D 提供存储地图，换算下坐标等
 * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
 */
class Costmap2D
{
  friend class CostmapTester;  // Need this for gtest to work correctly
public:
  /**
   * @brief  Constructor for a costmap
   * @param  cells_size_x The x size of the map in cells
   * @param  cells_size_y The y size of the map in cells
   * @param  resolution The resolution of the map in meters/cell
   * @param  origin_x The x origin of the map
   * @param  origin_y The y origin of the map
   * @param  default_value Default Value
   */  
  // 一副地图的起始点在左下角，origin是指该起始点在世界坐标系的坐标值。
  // 世界坐标系的原点是在建图时程序刚启动时的那个点。origin的值一般不为0,这样启动rviz显示就能看到机器人会在地图中相应的位置
  Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
            double origin_x, double origin_y, unsigned char default_value = 0);

  /**
   * @brief  Copy constructor for a costmap, creates a copy efficiently
   * @param map The costmap to copy 拷贝复制函数
   */
  Costmap2D(const Costmap2D& map);

  /**
   * @brief  Overloaded assignment operator
   * @param  map The costmap to copy
   * @return A reference to the map after the copy has finished
   */
  Costmap2D& operator=(const Costmap2D& map);

  /**
   * @brief  Turn this costmap into a copy of a window of a costmap passed in
   * @param  map The costmap to copy
   * @param win_origin_x The x origin (lower left corner) for the window to copy, in meters
   * @param win_origin_y The y origin (lower left corner) for the window to copy, in meters
   * @param win_size_x The x size of the window, in meters
   * @param win_size_y The y size of the window, in meters
   */
  bool copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
                         double win_size_y);

  /**
   * @brief  Default constructor
   */
  Costmap2D();

  /**
   * @brief  Destructor
   */
  virtual ~Costmap2D();

  /**
   * @brief  Get the cost of a cell in the costmap
   * @param mx The x coordinate of the cell
   * @param my The y coordinate of the cell
   * @return The cost of the cell
   */
  unsigned char getCost(unsigned int mx, unsigned int my) const;

  /**
   * @brief  Set the cost of a cell in the costmap
   * @param mx The x coordinate of the cell
   * @param my The y coordinate of the cell
   * @param cost The cost to set the cell to
   */
  void setCost(unsigned int mx, unsigned int my, unsigned char cost);

  /**
   * @brief  Convert from map coordinates to world coordinates
   * @param  mx The x map coordinate
   * @param  my The y map coordinate
   * @param  wx Will be set to the associated world x coordinate
   * @param  wy Will be set to the associated world y coordinate
   */
  void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

  /**
   * @brief  Convert from world coordinates to map coordinates
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @return True if the conversion was successful (legal bounds) false otherwise
   */
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

  /**
   * @brief  Convert from world coordinates to map coordinates without checking for legal bounds
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @note   The returned map coordinates <b>are not guaranteed to lie within the map.</b>
   */
  void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const;

  /**
   * @brief  Convert from world coordinates to map coordinates, constraining results to legal bounds.
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @note   The returned map coordinates are guaranteed to lie within the map.
   */
  void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const;

  /**
   * @brief  Given two map coordinates... compute the associated index，根据栅格坐标计算该栅格在costmap中的索引
   * @param mx The x coordinate
   * @param my The y coordinate
   * @return The associated index
   */
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return my * size_x_ + mx;
  }

  /**
   * @brief  Given an index... compute the associated map coordinates，根据栅格索引计算costmap栅格坐标
   * @param  index The index
   * @param  mx Will be set to the x coordinate
   * @param  my Will be set to the y coordinate
   */
  inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
  {
    my = index / size_x_;
    mx = index - (my * size_x_);
  }

  /**
   * @brief  Will return a pointer to the underlying unsigned char array used as the costmap
   * @return A pointer to the underlying unsigned char array storing cost values
   */
  unsigned char* getCharMap() const;

  /**
   * @brief  Accessor for the x size of the costmap in cells
   * @return The x size of the costmap
   */
  unsigned int getSizeInCellsX() const;

  /**
   * @brief  Accessor for the y size of the costmap in cells
   * @return The y size of the costmap
   */
  unsigned int getSizeInCellsY() const;

  /**
   * @brief  Accessor for the x size of the costmap in meters
   * @return The x size of the costmap (returns the centerpoint of the last legal cell in the map)
   */
  double getSizeInMetersX() const;

  /**
   * @brief  Accessor for the y size of the costmap in meters
   * @return The y size of the costmap (returns the centerpoint of the last legal cell in the map)
   */
  double getSizeInMetersY() const;

  /**
   * @brief  Accessor for the x origin of the costmap
   * @return The x origin of the costmap
   */
  double getOriginX() const;

  /**
   * @brief  Accessor for the y origin of the costmap
   * @return The y origin of the costmap
   */
  double getOriginY() const;

  /**
   * @brief  Accessor for the resolution of the costmap
   * @return The resolution of the costmap
   */
  double getResolution() const;

  void setDefaultValue(unsigned char c)
  {
    default_value_ = c;
  }

  unsigned char getDefaultValue()
  {
    return default_value_;
  }

  /**
   * @brief  Sets the cost of a convex polygon to a desired value
   * @param polygon The polygon to perform the operation on
   * @param cost_value The value to set costs to
   * @return True if the polygon was filled... false if it could not be filled
   */
  bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value);

  /**
   * @brief  Get the map cells that make up the outline of a polygon 获取构成多边形轮廓的地图栅格
   * @param polygon The polygon in map coordinates to rasterize，地图坐标系下
   * @param polygon_cells Will be set to the cells contained in the outline of the polygon
   */
  void polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);

  /**
   * @brief  Get the map cells that fill a convex polygon 获取填充凸多边形的地图栅格
   * @param polygon The polygon in map coordinates to rasterize
   * @param polygon_cells Will be set to the cells that fill the polygon
   */
  void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);

  /**
   * @brief  Move the origin of the costmap to a new location.... keeping data when it can
   * @param  new_origin_x The x coordinate of the new origin
   * @param  new_origin_y The y coordinate of the new origin
   */
  virtual void updateOrigin(double new_origin_x, double new_origin_y);

  /**
   * @brief  Save the costmap out to a pgm file
   * @param file_name The name of the file to save
   */
  bool saveMap(std::string file_name);

  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                 double origin_y);

  void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);

  /**
   * @brief  Given distance in the world... convert it to cells
   * @param  world_dist The world distance
   * @return The equivalent cell distance，世界坐标系下的距离转换为costmap栅格距离
   */
  unsigned int cellDistance(double world_dist);

  // Provide a typedef to ease future code maintenance
  typedef boost::recursive_mutex mutex_t;
  mutex_t* getMutex()
  {
    return access_;
  }

protected:
  /**
   * @brief  Copy a region of a source map into a destination map
   * @param  source_map The source map
   * @param sm_lower_left_x The lower left x point of the source map to start the copy，源map左下角
   * @param sm_lower_left_y The lower left y point of the source map to start the copy
   * @param sm_size_x The x size of the source map
   * @param  dest_map The destination map
   * @param dm_lower_left_x The lower left x point of the destination map to start the copy，目标map左下角
   * @param dm_lower_left_y The lower left y point of the destination map to start the copy
   * @param dm_size_x The x size of the destination map
   * @param region_size_x The x size of the region to copy
   * @param region_size_y The y size of the region to copy
   */
  template<typename data_type>
    void copyMapRegion(data_type* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, data_type* dest_map, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y)
    {
      // we'll first need to compute the starting points for each map
      data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
      data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

      // now, we'll copy the source map into the destination map
      for (unsigned int i = 0; i < region_size_y; ++i)
      {
        memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
        sm_index += sm_size_x;
        dm_index += dm_size_x;
      }
    }

  /**
   * @brief  Deletes the costmap, static_map, and markers data structures
   */
  virtual void deleteMaps();

  /**
   * @brief  Resets the costmap and static_map to be unknown space
   */
  virtual void resetMaps();

  /**
   * @brief  Initializes the costmap, static_map, and markers data structures
   * @param size_x The x size to use for map initialization
   * @param size_y The y size to use for map initialization
   */
  virtual void initMaps(unsigned int size_x, unsigned int size_y);

  /**
   * @brief  Raytrace a line and apply some action at each step
   * @param  at The action to take... a functor
   * @param  x0 The starting x coordinate
   * @param  y0 The starting y coordinate
   * @param  x1 The ending x coordinate
   * @param  y1 The ending y coordinate
   * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
   */
  /*算法流程伪代码
  * function line(x0, y0, x1, y1)
  *   real deltax := x1 - x0
  *   real deltay := y1 - y0
  *   real k := abs(deltay / deltax)
  *   real error := 0.0 // No error at start
  *   int y := y0
  *   for x from x0 to x1 
  *   {   
  *       plot(x,y)
  *       error := error + k
  *       if error ≥ 0.5 then
  *        { 
  *          y := y + sign(deltay)
  *          error := error - 1.0
  *        }
  *    }
  */
  template<class ActionType>
    inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                             unsigned int max_length = UINT_MAX)
    {
      int dx = x1 - x0;
      int dy = y1 - y0;

      unsigned int abs_dx = abs(dx);
      unsigned int abs_dy = abs(dy);

      int offset_dx = sign(dx);
      int offset_dy = sign(dy) * size_x_;

      unsigned int offset = y0 * size_x_ + x0;

      // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
      double dist = hypot(dx, dy);
      double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist); // 长度不超过max_length

      // if x is dominant
      if (abs_dx >= abs_dy)
      {
        int error_y = abs_dx / 2;
        bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        return;
      }

      // otherwise y is dominant
      int error_x = abs_dy / 2;
      bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
    }

private:
  /**
   * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
   * bresenham算法实现了对于离散的平面点，指定两个端点，找到两点之间的其他像素点，使得这些像素点组成一个尽可能趋近直线的点集
   */
  template<class ActionType>
    inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                            int offset_b, unsigned int offset, unsigned int max_length)
    {
      unsigned int end = std::min(max_length, abs_da);
      for (unsigned int i = 0; i < end; ++i)
      {
        at(offset); //最先保存初始坐标(x0,y0)
        offset += offset_a; //x+1
        error_b += abs_db; //相当于在两边同乘了dx，d(i+1)=d(i)+k => d(i+1)*dx=d(i)*dx+dy。d(0)=0+1/2
        if ((unsigned int)error_b >= abs_da) //因为在初始值时就加了1/2，本来应该是判别1/2的，减少了除法运算。意义是直线上的点离待判定y轴坐标的差值
        {
          offset += offset_b; //y+1
          error_b -= abs_da; //如果出现了负值也没关系，则保持上一次的y值
        }
      }
      at(offset);
    }

  inline int sign(int x)
  {
    return x > 0 ? 1.0 : -1.0;
  }

  mutex_t* access_;  // 信号量
protected:
  unsigned int size_x_;           // x轴尺寸, The x size of the map in cells
  unsigned int size_y_;           // y轴尺寸, The y size of the map in cells
  double resolution_;             // 分辨率, The resolution of the map in meters/cell
  double origin_x_;               // x轴原点坐标(world坐标)
  double origin_y_;               // y轴原点坐标(world坐标)
  unsigned char* costmap_;        // 存储空间， costmap_ = new unsigned char[size_x * size_y] ，里面存的就是代价值
  unsigned char default_value_;   // 默认填充值

  class MarkCell
  {
  public:
    MarkCell(unsigned char* costmap, unsigned char value) :
        costmap_(costmap), value_(value)
    {
    }
    inline void operator()(unsigned int offset)
    {
      costmap_[offset] = value_;
    }
  private:
    unsigned char* costmap_;
    unsigned char value_;
  };

  class PolygonOutlineCells
  {
  public:
    PolygonOutlineCells(const Costmap2D& costmap, const unsigned char* char_map, std::vector<MapLocation>& cells) :
        costmap_(costmap), char_map_(char_map), cells_(cells)
    {
    }

    // just push the relevant cells back onto the list
    inline void operator()(unsigned int offset)
    {
      MapLocation loc;
      costmap_.indexToCells(offset, loc.x, loc.y);
      cells_.push_back(loc);
    }

  private:
    const Costmap2D& costmap_;
    const unsigned char* char_map_;
    std::vector<MapLocation>& cells_;
  };
};
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_H
