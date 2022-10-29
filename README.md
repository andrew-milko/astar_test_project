# A* search algorithm

In this repository you can find C++ implementation of `A* search algorithm`.

## Problem statement
- 2D-map consists of `n x n` cells
- white cells are `free`, black cells are `obstacles` (you can't build paths through them)
- blue cells refer to the optimal path

## Two approaches for visualization
- `2d_picture.png`: image with 2D-grid of obstacles and optimal path
- `foxglove_topic.bag`: special file format for [Foxglove Studio](https://foxglove.dev/) to recreate optimal path in 3D environment

## Map format
Map is stored in a file `map.txt`, for ex.:

```
start_x end_x
end_x end_y
n
x_1 y_1
x_2 y_2

...

x_k y_k
```

- `start_x, end_x` - coordinates of starting point
- `end_x, end_y` - coordinates of starting point
- `n` - dimension of map
- `x_i, y_i` - coordinats of obstacle points

## Map generation
For generating custom map by yourself, you need to set the following parameters:
- `start_x, end_x` - coordinates of starting point
- `end_x, end_y` - coordinates of starting point
- `n` - dimension of map
- `obstacles_density` - how many obstacles will be on the map (from 0.0 to 1.0)
