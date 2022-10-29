#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <set>

using namespace std;

class Node {
    public:
        int x;
        int y;
        int obstacle = 0;
        float g = 0.0;
        float h = 0.0;
        float f = 0.0;
        vector<Node*> neighbors; 
        Node* prev_node = nullptr;

        bool operator< (const Node& e) const {
            return (x < e.x) || (x == e.x && y < e.y);
        }

        void add_neighbors(int n, vector<vector<Node>>& nodes) {
            if (x > 0)
                neighbors.push_back(&nodes[x - 1][y]);
            if (x < n - 1)
                neighbors.push_back(&nodes[x + 1][y]);
            if (y > 0) 
                neighbors.push_back(&nodes[x][y - 1]);
            if (y < n - 1)
                neighbors.push_back(&nodes[x][y + 1]);
            if ((x > 0) && (y < n - 1))
                neighbors.push_back(&nodes[x - 1][y + 1]);
            if ((x < n - 1) && (y < n - 1))
                neighbors.push_back(&nodes[x + 1][y + 1]);
            if ((x > 0) && (y > 0))
                neighbors.push_back(&nodes[x - 1][y - 1]);
            if ((x < n - 1) && (y > 0))
                neighbors.push_back(&nodes[x + 1][y - 1]);
        }

        Node() {}

        Node (int x_, int y_) {
            x = x_;
            y = y_;
        }
};

float nodes_dist(const Node& current_node, const Node& neighbor) {
    if ((current_node.x != neighbor.x) && (current_node.y != neighbor.y))
        return sqrt(2);
    else
        return 1.0;
}

float heuristic(const Node& neighbor, const Node& end_node) {
    return sqrt(pow((neighbor.x - end_node.x), 2) + pow((neighbor.y - end_node.y), 2));
}


void initialize_nodes(int n, vector<vector<int>>& obstacles , vector<vector<Node>>& nodes) {
    nodes = vector<vector<Node>>(n, vector<Node>(n));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            nodes[i][j] = Node(i, j);
            nodes[i][j].obstacle = obstacles[i][j];
        }
    }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            nodes[i][j].add_neighbors(n, nodes);
        }
    }
}

void generate_map(vector<vector<int>> &obstacles, int n, int start_x, int start_y, int end_x, int end_y, float obstacles_density) {
    obstacles = vector<vector<int>>(n, vector<int>(n, 0));

    ofstream out;         
    out.open("map.txt");

    out << start_x << " " << start_y << "\n";
    out << end_x << " " << end_y << "\n";
    out << n << "\n"; 

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            if (r < obstacles_density) {
                if ((i == start_x) && (j == start_y)) {
                    // do nothing
                } else if ((i == end_x) && (j == end_y)) {
                    // do nothing
                } else {
                    obstacles[i][j] = 1;
                    out << i << " " << j << "\n";
                }
            }
        }
    }

    out.close();
}

void generate_path(vector<pair<int, int>>& path_coords, Node& current_node) {
    path_coords.push_back(make_pair(current_node.x, current_node.y));

    while (current_node.prev_node != nullptr) {
        current_node = *current_node.prev_node;
        path_coords.push_back(make_pair(current_node.x, current_node.y));
    }
}

void upload_path(const vector<pair<int, int>>& path_coords) {
    ofstream out;         
    out.open("path.txt");

    for (auto pair : path_coords) {
        out << pair.first << " " << pair.second << "\n";
    } 

    out.close();
}


void find_path(vector<pair<int, int>>& path_coordinates, bool read_map=true, int n=100, int start_x=0, int start_y=0, int end_x=0, int end_y=0, float obstacles_density=0.3) {
    vector<vector<Node>> nodes;
    vector<vector<int>> obstacles;

    // Map creation
    if (read_map) {
        fstream myfile("map.txt");
        
        myfile >> start_x >> start_y;
        myfile >> end_x >> end_y;
        myfile >> n;

        obstacles = vector<vector<int>>(n, vector<int>(n, 0));

        int x, y, tmp;

        while (myfile >> tmp) {
            x = tmp;
            myfile >> tmp;
            y = tmp;
            obstacles[x][y] = 1;
        }

        myfile.close();
        
    } else {
        generate_map(obstacles, n, start_x, start_y, end_x, end_y, obstacles_density);
    }

    // Initialization
    initialize_nodes(n, obstacles, nodes);

    // Create start node, end node, open set and closed set
    Node& start_node = nodes[start_x][start_y];
    Node& end_node = nodes[end_x][end_y];
    set<Node*> open_set, closed_set;
    open_set.insert(&start_node);
    
    // Path finding
    while (open_set.size() > 0) {
        vector<Node*> open_set_vector;
        copy(open_set.begin(), open_set.end(), std::back_inserter(open_set_vector));

        int lowest_index = 0;
        for (int i = 0; i < open_set_vector.size(); i++) {
            if (open_set_vector[i]->f < open_set_vector[lowest_index]->f) {
                lowest_index = i;
            }
        }

        Node& current_node = *(open_set_vector[lowest_index]);

        if ((current_node.x == end_node.x) && (current_node.y == end_node.y)) {
            generate_path(path_coordinates, current_node);
            upload_path(path_coordinates);
            break;
        }

        closed_set.insert(&current_node);
        open_set.erase(&current_node);

        for (auto neighbor : current_node.neighbors) {
            bool neighbor_changed = false;

            if ((closed_set.find(neighbor) == closed_set.end()) && (neighbor->obstacle == 0)) {
                float temp_g = current_node.g + nodes_dist(current_node, *neighbor);

                if (open_set.find(neighbor) != open_set.end()) {
                    if (temp_g < neighbor->g) {
                        neighbor_changed = true;
                        neighbor->g = temp_g;
                    }
                } else {
                    neighbor_changed = true;
                    neighbor->g = temp_g;

                    open_set.insert(neighbor);
                }
            }

            if (neighbor_changed == true) {
                neighbor->h = heuristic(*neighbor, end_node);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->prev_node = &current_node;
            }
        }
    }

    return;
}

int main() {
    bool read_map = true;
    vector<pair<int, int>> path_coordinates;
    
    if (read_map) {
        find_path(path_coordinates, true);
    } else {
        find_path(path_coordinates, false, 1000, 200, 100, 700, 900, 0.35);
    }
    
    if (path_coordinates.size() > 0) {
        cout << "Optimal path was found and saved to 'path.txt' file.\n";
    } else {
        cout << "Optimal path doesn't exist for current problem statement.\n";
    }

    return 0;
}