#include <iostream>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace ctr{

const double w_gain = 1.0;

class a_star
{
public:
    a_star(double resolution_, double robotRadius_);
    bool planning(std::vector<double>& fx, std::vector<double>& fy, double sx, double sy, double gx, double gy, const std::vector<double> ox, const std::vector<double> oy);


private:
    struct Node
    {
        int x;
        int y;
        double sum_cost;
        Node* p_node = NULL;
    };

    std::vector<Node> get_motion_model();
    std::vector<std::vector<int>> calc_obstacle_map(const std::vector<int>& ox, const std::vector<int>& oy, const int min_ox, const int max_ox, const int min_oy, const int max_oy);
    bool verify_node(Node* node, std::vector<std::vector<int>> obmap, int min_ox, int max_ox, int min_oy, int max_oy);
    void calc_final_path(Node* goal, std::vector<double>& fx, std::vector<double>& fy);

    double resolution;
    double robotRadius;
};

a_star::a_star(double resolution_, double robotRadius_) : resolution(resolution_), robotRadius(robotRadius_)
{

}

bool a_star::planning(std::vector<double>& fx, std::vector<double>& fy, double sx, double sy, double gx, double gy, const std::vector<double> ox, const std::vector<double> oy)
{
    Node* nstart = new Node{(int)std::round(sx/resolution), (int)std::round(sy/resolution), 0.0};
    Node* ngoal = new Node{(int)std::round(gx/resolution), (int)std::round(gy/resolution), 0.0};

    int min_ox = std::numeric_limits<int>::max();
    int max_ox = std::numeric_limits<int>::min();
    int min_oy = std::numeric_limits<int>::max();
    int max_oy = std::numeric_limits<int>::min();

    //position(m) -> node index
    std::vector<int> ox_idx;
    std::vector<int> oy_idx;

    for(const auto& iox:ox){
        int map_x = (int)std::round(iox*1.0/resolution);
        ox_idx.push_back(map_x);
        min_ox = std::min(map_x, min_ox);
        max_ox = std::max(map_x, max_ox);
    }

    for(const auto& ioy:oy){
        int map_y = (int)std::round(ioy*1.0/resolution);
        oy_idx.push_back(map_y);
        min_oy = std::min(map_y, min_oy);
        max_oy = std::max(map_y, max_oy);
    }

    int xwidth = max_ox-min_ox;
    int ywidth = max_oy-min_oy;

    std::vector<std::vector<int>> visit_map(xwidth, std::vector<int>(ywidth, 0));
    std::vector<std::vector<double>> path_cost(xwidth, std::vector<double>(ywidth, std::numeric_limits<double>::max()));

    path_cost[nstart->x][nstart->y] = 0;

    std::vector<std::vector<int> > obmap = calc_obstacle_map(ox_idx, oy_idx, min_ox, max_ox, min_oy, max_oy);

    auto cmp = [](const Node* left, const Node* right){return left->sum_cost > right->sum_cost;};
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> pq(cmp);
    pq.push(nstart);

    std::vector<Node> motion = get_motion_model();

    while(true)
    {
        if(pq.empty()){
            std::cout<<"path don't find!"<<std::endl;
            return false;
        }
        Node* node = pq.top();

        if(visit_map[node->x-min_ox][node->y-min_oy] == 1){
            //searched area
            pq.pop();
            delete node;
            continue;
        }else{
            //unsearched area
            pq.pop();
            visit_map[node->x-min_ox][node->y-min_oy] = 1;
        }

        //goal
        if(node->x == ngoal->x && node->y == ngoal->y){
            ngoal->sum_cost = node->sum_cost;
            ngoal->p_node = node;
            break;
        }

        for(int i=0; i<motion.size(); i++){
            //search new node according to motion model
            Node* new_node = new Node{node->x + motion[i].x, node->y + motion[i].y,
                           path_cost[node->x][node->y] + motion[i].sum_cost + w_gain*sqrt(pow(ngoal->x - node->x, 2) + pow(ngoal->y - node->y, 2)),
                           node};

            //avoid obstract area
            if (!verify_node(new_node, obmap, min_ox, max_ox, min_oy, max_oy)){
                delete new_node;
                continue;
            }

            //searched area
            if (visit_map[new_node->x-min_ox][new_node->y-min_oy]){
                delete new_node;
                continue;
            }

            //select min cost path
            if (path_cost[node->x][node->y]+motion[i].sum_cost < path_cost[new_node->x][new_node->y]){
                path_cost[new_node->x][new_node->y]=path_cost[node->x][node->y]+motion[i].sum_cost;
                pq.push(new_node);
            }
        }
    }


    calc_final_path(ngoal, fx, fy);
    delete ngoal;
    delete nstart;

    return true;

}

std::vector<a_star::Node> a_star::get_motion_model()
{
    return {Node{1, 0, 1},
            Node{0, 1, 1},
            Node{-1, 0, 1},
            Node{0, -1, 1},
            Node{-1, -1, sqrt(2)},
            Node{-1, 1, sqrt(2)},
            Node{1, -1, sqrt(2)},
            Node{1, 1, sqrt(2)}};
}

std::vector<std::vector<int>> a_star::calc_obstacle_map(const std::vector<int>& ox, const std::vector<int>& oy, const int min_ox, const int max_ox, const int min_oy, const int max_oy)
{
    int xwidth = max_ox-min_ox;
    int ywidth = max_oy-min_oy;
    std::vector<std::vector<int>> obmap(ywidth, std::vector<int>(xwidth, 0));

    for(int i=0; i<xwidth; i++){
        int x = i + min_ox;
        for(int j=0; j<ywidth; j++){
            int y = j + min_oy;
            for(int k=0; k<ox.size(); k++){
                double d = sqrt(pow(ox[k]-x, 2) + pow(oy[k]-y, 2));

                if(d <= robotRadius/resolution){
                    obmap[i][j] = 1;
                }
            }
        }
    }

    return obmap;
}

bool a_star::verify_node(Node* node, std::vector<std::vector<int>> obmap, int min_ox, int max_ox, int min_oy, int max_oy)
{
    if(node->x < min_ox || node->y < min_oy || node->x >= max_ox || node->y >= max_oy){
        return false;
    }
    if(obmap[node->x - min_ox][node->y - min_oy]){
        return false;
    }

    return true;
}

void a_star::calc_final_path(Node* goal, std::vector<double>& fx, std::vector<double>& fy)
{
    std::vector<double> rx;
    std::vector<double> ry;
    Node* node = goal;

    while(node->p_node != NULL)
    {
        node = node->p_node;
        rx.push_back(node->x * resolution);
        ry.push_back(node->y * resolution);
    }

    fx = rx;
    fy = ry;
}

}