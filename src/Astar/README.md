### A*算法原理
全局路径规划算法，根据给定的起点和终点在全局地图上进行总体路径规划。
>导航中使用A*算法计算出机器人到目标位置的最优路线，一般作为规划的参考路线

![在这里插入图片描述](https://img-blog.csdnimg.cn/d24b6de0e3124ac8a165c181e4084c61.png)

```cpp
// 定义地图上的点
struct Point
{
    int x,y; // 栅格行列
    Point(int x, int y):x(x),y(y){}; // 参数列表初始化
    double distance(Point& p)        // 求距离
    {
        return sqrt((x-p.x)*(x-p.x)+(y-p.y)*(y-p.y)); // 欧几里得距离
    }
};
```

```cpp
// 定义节点
struct Node
{
    Point point; // 栅格点
    double g,h,f;// 代价值,f总价值,g到起点的代价值，h到终点的估计代价（启发式函数）
    Node *parent;// 父节点指针
    Node(Point point, double g, double h, Node* parent = nullptr):point(point), g(g), h(h), f(g+h), parent(parent)
    {}
};
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/eeda7e73438b4073bbb1d902c4eb0f9a.png)

```cpp
// 定义地图
 vector<vector<int>> gridmap = {
         {0, 1, 0, 0, 0},
         {0, 0, 1, 0, 0},
         {0, 0, 1, 1, 0},
         {0, 0, 1, 0, 0},
         {0, 0, 1, 1, 0}
 };
 // 定义起点和终点
 Point start{0, 0};
 Point goal{4, 4};
```
A*算法的寻路原理
![在这里插入图片描述](https://img-blog.csdnimg.cn/1df065a278a347cca17aa0efe4e9a2f1.png)


A*的结束条件
![在这里插入图片描述](https://img-blog.csdnimg.cn/8f0442dedc564c69bd75f8abf93f5d04.png)
A*算法的寻路详细步骤
![在这里插入图片描述](https://img-blog.csdnimg.cn/04e9bc39114f405a813b0a0154def770.png)
![在这里插入图片描述](https://img-blog.csdnimg.cn/8f8e661706244e69a8a2d0a6966829e8.png)