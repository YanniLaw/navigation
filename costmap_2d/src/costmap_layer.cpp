#include<costmap_2d/costmap_layer.h>

namespace costmap_2d
{

void CostmapLayer::touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y)
{
    *min_x = std::min(x, *min_x);
    *min_y = std::min(y, *min_y);
    *max_x = std::max(x, *max_x);
    *max_y = std::max(y, *max_y); // 由形参指定的边界框，要包含(x,y)
}

void CostmapLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap(); // Layer继承过来的LayeredCostmap* layered_costmap_，主图层
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
    // 这里调整的对象是继承的Costmap2D的数据成员char* costmap_, 也就是这里的调整是针对各层的地图，不是关于master map的。
}

// 清图，将指定区域的栅格变为未知区域栅格
void CostmapLayer::clearArea(int start_x, int start_y, int end_x, int end_y, bool invert_area)
{
  unsigned char* grid = getCharMap();
  for(int x=0; x<(int)getSizeInCellsX(); x++){
    bool xrange = x>start_x && x<end_x;

    for(int y=0; y<(int)getSizeInCellsY(); y++){
      if((xrange && y>start_y && y<end_y)!=invert_area)
        continue;
      int index = getIndex(x,y);
      if(grid[index]!=NO_INFORMATION){
        grid[index] = NO_INFORMATION;
      }
    }
  }
}

void CostmapLayer::addExtraBounds(double mx0, double my0, double mx1, double my1)
{
    extra_min_x_ = std::min(mx0, extra_min_x_);
    extra_max_x_ = std::max(mx1, extra_max_x_);
    extra_min_y_ = std::min(my0, extra_min_y_);
    extra_max_y_ = std::max(my1, extra_max_y_);
    has_extra_bounds_ = true;
}

void CostmapLayer::useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!has_extra_bounds_) // 若没有调用addExtraBounds函数
        return;

    *min_x = std::min(extra_min_x_, *min_x);
    *min_y = std::min(extra_min_y_, *min_y);
    *max_x = std::max(extra_max_x_, *max_x);
    *max_y = std::max(extra_max_y_, *max_y); // 由形参指定的边界框，要包含调用addExtraBounds函数后确定的边界框
    extra_min_x_ = 1e6;
    extra_min_y_ = 1e6;
    extra_max_x_ = -1e6;
    extra_max_y_ = -1e6;
    has_extra_bounds_ = false;
}

void CostmapLayer::updateWithMax(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  // costmap_是本层的地图数据。直接将本层的数据赋值给master map的对应索引就行了
  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      if (costmap_[it] == NO_INFORMATION){ // 若本层的值是NO_INFORMATION，则主网格的值不改变
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION || old_cost < costmap_[it]) // 若主网格的值是NO_INFORMATION，或设置为主网格的值和本层的值的最大值
        master_array[it] = costmap_[it];
      it++;
    }
  }
}

// costmap_是本层的地图数据。直接将本层的数据赋值给master map的对应索引就行了
void CostmapLayer::updateWithTrueOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                           int max_i, int max_j)
{
  if (!enabled_)
    return;
  unsigned char* master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = span*j+min_i;
    for (int i = min_i; i < max_i; i++)
    {
      master[it] = costmap_[it]; // 强制性的使用本层的值，在指定的边界框中更新主网格中的值
      it++;
    }
  }
}

void CostmapLayer::updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  unsigned char* master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  // costmap_是本层的地图数据。直接将本层的数据赋值给master map的对应索引就行了
  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = span*j+min_i;
    for (int i = min_i; i < max_i; i++)
    {
      if (costmap_[it] != NO_INFORMATION) // 在指定的边界框中更新主网格中的值(只更新有效值)
        master[it] = costmap_[it];
      it++;
    }
  }
}

void CostmapLayer::updateWithAddition(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  // costmap_是本层的地图数据。直接将本层的数据赋值给master map的对应索引就行了
  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      if (costmap_[it] == NO_INFORMATION){ // 若本层的值是NO_INFORMATION，则主网格的值不改变
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION) // 如果主网格的值是NO_INFORMATION，则直接用本层的值覆盖它
        master_array[it] = costmap_[it];
      else
      {
        int sum = old_cost + costmap_[it]; // 设置为主网格的值和本层的值的和
        if (sum >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) // 内切圆的障碍代价值
            master_array[it] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
        else
            master_array[it] = sum;
      }
      it++;
    }
  }
}
}  // namespace costmap_2d
