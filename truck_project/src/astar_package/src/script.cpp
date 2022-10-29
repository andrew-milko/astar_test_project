#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

using namespace std;

void initialize_cube_list(const rclcpp::Node& node, visualization_msgs::msg::Marker& cube_list) {
    cube_list.header.frame_id = "/frame_name";
    cube_list.header.stamp = node.now();

    cube_list.id = 1;

    cube_list.action = visualization_msgs::msg::Marker::ADD;
    cube_list.type = visualization_msgs::msg::Marker::CUBE_LIST;

    cube_list.scale.x = 1;
    cube_list.scale.y = 1;
    cube_list.scale.z = 0.2;

    cube_list.color.a = 1.0;
    cube_list.color.r = 1.0;
    cube_list.color.g = 1.0;
    cube_list.color.b = 1.0;
}

void initialize_line_strip(const rclcpp::Node& node, visualization_msgs::msg::Marker& line_strip) {
    line_strip.header.frame_id = "/frame_name";
    line_strip.header.stamp  = node.now();

    line_strip.id = 0;

    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;

    line_strip.scale.x = 0.5;
    line_strip.scale.y = 0.5;

    line_strip.color.a = 1.0;
    line_strip.color.r = 0.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 1.0;
}

void read_files(const rclcpp::Node& node, vector<pair<int, int>>& obstacles_coordinates, vector<pair<int, int>>& path_coordinates) {
    int tmp;
    RCLCPP_INFO(node.get_logger(), "Read path coordinates.");
    std::fstream path_file("/home/andrew/Desktop/truck_project/src/astar_package/src/path.txt");

    while (path_file >> tmp) {
        int x = tmp;
        path_file >> tmp;
        int y = tmp;

        path_coordinates.push_back(make_pair(x, y));
    }

    path_file.close();

    int s_x, s_y, e_x, e_y, n;
    RCLCPP_INFO(node.get_logger(), "Read obstacles coordinates.");
    std::fstream obstacles_file("/home/andrew/Desktop/truck_project/src/astar_package/src/map.txt");

    obstacles_file >> s_x >> s_y >> e_x >> e_y >> n;

    while (obstacles_file >> tmp) {
        int x = tmp;
        obstacles_file >> tmp;
        int y = tmp;

        obstacles_coordinates.push_back(make_pair(x, y));
    }

    obstacles_file.close();
}

void assign_positions_to(const rclcpp::Node& node, const vector<pair<int, int>>& coordinates, visualization_msgs::msg::Marker& marker) {
    RCLCPP_INFO(node.get_logger(), "Create figures based on coordinates.");

    for (uint32_t i = 0; i < coordinates.size(); i++) {
        geometry_msgs::msg::Point p;

        p.x = coordinates[i].first;
        p.y = coordinates[i].second;
        p.z = 0;

        marker.points.push_back(p);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node node("node_name");
    
    auto publisher= node.create_publisher<visualization_msgs::msg::Marker>("/topic_name", 1);
    RCLCPP_INFO(node.get_logger(), "Publisher is created.");

    // LINE_STRIP
    visualization_msgs::msg::Marker line_strip;
    initialize_line_strip(node, line_strip);

    // CUBE_LIST
    visualization_msgs::msg::Marker cube_list;
    initialize_cube_list(node, cube_list);

    // Read files
    vector<pair<int, int>> obstacles_coordinates;
    vector<pair<int, int>> path_coordinates;
    read_files(node, obstacles_coordinates, path_coordinates);

    assign_positions_to(node, path_coordinates, line_strip);
    assign_positions_to(node, obstacles_coordinates, cube_list);

    publisher->publish(line_strip);
    publisher->publish(cube_list);

    while(rclcpp::ok());

    rclcpp::shutdown();

    return 0;
}