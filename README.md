# A* search algorithm

In this repository you can find C++ implementation of `A* search algorithm`.

![My Image](readme_files/foxglove_readme_ex.png)

## Problem statement
- map is a 2D-grid which consists of `n x n` cells
- white cells are `free`, black cells are `obstacles` (you can't build paths through them)
- blue cells refer to the optimal path

## Map format
Data about 2D-grid is stored in a file `map.txt` with following structure:

```
start_x end_x
end_x end_y
n
x_1 y_1
x_2 y_2

...

x_k y_k
```

- `start_x, start_y` - coordinates of starting point
- `end_x, end_y` - coordinates of ending point
- `n` - dimension of map
- `x_i, y_i` - coordinates of obstacle points

## Map generation
For generating custom 2D-grid by yourself (creating `map.txt` file), you need to set the following parameters:
- `start_x, end_x` - coordinates of starting point
- `end_x, end_y` - coordinates of ending point
- `n` - dimension of map
- `obstacles_density` - how many obstacles will be on the map *(from 0.0 to 1.0)*

## Path format
If optimal path was found, it's stored in a file `path.txt` with following structure:

```
x_1 y_1
x_2 y_2

...

x_k y_k
```

- `x_i, y_i` - coordinates of optimal path's points

## Two approaches for visualization
- `2d_picture.png`: image of 2D-grid with obstacles and optimal path ([Google Colab Notebook](https://colab.research.google.com/drive/1ivw2n-Ior9ehzbsOHj1cHsbU3BGrJef_?usp=sharing))
- `3d_foxglove.db3`: special file format for [Foxglove Studio](https://foxglove.dev/) to recreate found optimal path in 3D environment

## Perfomance
In my opinion, the code `astar.cpp` is written optimally (as far as my knowledge allows). I tried not to make unnecessary copies of data, but to use links and poinetrs. I think the code can be accelerated if you experiment with different data types. I made some tests to find out the average time of finding the optimal path `(~4.8 sec.)` depending on different `n` and `obstacles_density` values. Results are stored in `perfomance_log.txt` file.

## Prerequisites
- `Ubuntu 20.04`
- `ROS 2 Galactic`
