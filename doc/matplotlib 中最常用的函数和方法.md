在 `matplotlib` 中，有很多常用的函数和方法用于绘制和调整图形。以下是一些常见的函数，按用途分类进行了介绍：

### 1. **创建和管理图形**
- `plt.figure()`: 创建一个新的图形窗口，可以设置图形的大小和分辨率。
- `plt.subplot()`: 在同一个图形窗口中创建多个子图（网格布局）。
- `plt.subplots()`: 创建一组子图，常用来一次性生成多个子图对象。
- `plt.gca()`: 获取当前的坐标轴对象。
- `plt.gcf()`: 获取当前的图形对象。
- `plt.clf()`: 清空当前图形。
- `plt.close()`: 关闭图形窗口。

### 2. **绘图函数**
- `plt.plot(x, y)`: 创建 2D 线图，x 和 y 是数据。
- `plt.scatter(x, y)`: 创建散点图。
- `plt.bar(x, height)`: 创建柱状图。
- `plt.hist(data, bins)`: 绘制直方图，`bins` 定义分区数量。
- `plt.pie()`: 绘制饼状图。
- `plt.imshow()`: 显示图像，通常用于处理二维数组（如图像数据）。
- `plt.contour()`: 绘制等高线图。
- `plt.fill_between()`: 在两条曲线之间填充颜色。
- `plt.errorbar()`: 绘制带有误差条的线图。
- `plt.stem()`: 绘制脉冲或离散数据图。

### 3. **三维绘图函数** (需要 `mpl_toolkits.mplot3d`)
- `ax = plt.axes(projection='3d')`: 创建 3D 坐标轴。
- `ax.plot3D(x, y, z)`: 创建 3D 线图。
- `ax.scatter3D(x, y, z)`: 创建 3D 散点图。
- `ax.contour3D(x, y, z)`: 创建 3D 等高线图。
- `ax.bar3D(x, y, z, dx, dy, dz)`: 创建 3D 柱状图。

### 4. **添加标题和标签**
- `plt.title()`: 添加标题。
- `plt.xlabel()`: 添加 X 轴标签。
- `plt.ylabel()`: 添加 Y 轴标签。
- `plt.zlabel()`: 添加 Z 轴标签（3D 图形时）。
- `plt.legend()`: 显示图例。
- `plt.text(x, y, s)`: 在指定位置添加文本。
- `plt.annotate()`: 更高级的注释文本函数，支持箭头等标记。

### 5. **调整坐标轴**
- `plt.xlim()`: 设置 X 轴的范围。
- `plt.ylim()`: 设置 Y 轴的范围。
- `plt.xticks()`: 设置 X 轴的刻度标签。
- `plt.yticks()`: 设置 Y 轴的刻度标签。
- `plt.grid()`: 显示网格线。
- `plt.axhline()`: 添加水平线。
- `plt.axvline()`: 添加垂直线。
- `plt.axis()`: 设置或获取当前图形的轴范围，例如 `plt.axis('equal')` 可以使 X 轴和 Y 轴单位比例一致。

### 6. **颜色和样式**
- `plt.colormaps()`: 获取所有可用的颜色图（colormaps）。
- `plt.set_cmap()`: 设置颜色图（用于 `imshow` 等函数）。
- `plt.style.use()`: 使用特定的绘图样式（如 `ggplot`, `seaborn` 等）。
- `plt.rcParams`: 用于全局设置图形属性，例如字体大小、图例位置等。

### 7. **保存图像**
- `plt.savefig()`: 保存图形到文件，支持多种格式如 PNG, PDF, SVG 等。

### 8. **交互与动态**
- `plt.pause(interval)`: 暂停一段时间，用于动画或动态显示。
- `plt.draw()`: 手动刷新图像，用于动态更新。

### 9. **显示图像**
- `plt.show()`: 显示图像窗口。如果你在绘制完图形后没有调用 `plt.show()`，图像可能不会弹出显示。

### 10. **其他**
- `plt.tight_layout()`: 自动调整子图布局，避免重叠。
- `plt.subplots_adjust()`: 手动调整子图布局和间距。
