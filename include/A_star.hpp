#include <stdio.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "struct.hpp"

class Node
{
   public:
    Pose pose;

    double g_cost;
    double h_cost;
    double f_cost;

    Node* parent;

    bool operator==(const Node& node);
};

bool Node::operator==(const Node& node)
{
    return (node.pose.x == this->pose.x && node.pose.y == this->pose.y);
}

class AStar
{
   public:
    AStar(Pose departure, Pose goal, std::pair<int, int> map_size, int density);
    ~AStar()
    {
        // Destructor code here
    }
    void InitializeNode();
    bool Run();
    std::vector<Node> PropagationNode(Node* current_node);

    bool IsOnList(Node node, std::vector<Node*> list);
    bool IsNodeOnGoal();
    void PrintMap();

    double ComputeActualCost(Node node, Pose pose_n);
    double ComputeHeuristicCost(Pose pose_n);

    void ReconstructPath(Node* current_node);

    std::vector<std::pair<int, int>> move = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

    Pose m_departure;
    Pose m_goal;

    std::vector<std::vector<char>> m_map;
    int m_width;
    int m_height;

    std::vector<Node*> m_open_list;
    std::vector<Node*> m_close_list;

    std::vector<Node*> m_path;
};

AStar::AStar(Pose departure, Pose goal, std::pair<int, int> map_size, int density)
{
    m_departure = departure;
    m_goal = goal;
    m_width = map_size.first;
    m_height = map_size.second;

    srand(time(NULL));

    for (int i = 0; i < m_height; i++)
    {
        std::vector<char> row;

        for (int j = 0; j < m_width; j++)
        {
            if (int(rand() % m_height) < density)
                row.push_back('o');  // obstacle
            else
                row.push_back('.');
        }

        m_map.push_back(row);
    }
    m_map[m_departure.x][m_departure.y] = '@';
    m_map[m_goal.x][m_goal.y] = 'x';

    InitializeNode();
}

void AStar::PrintMap()
{
    for (int i = 0; i < m_height; i++)
    {
        for (int j = 0; j < m_width; j++)
        {
            std::cout << m_map[i][j];
        }
        std::cout << std::endl;
    }
}

double AStar::ComputeActualCost(Node current_node, Pose pose_n)
{
    Node node = current_node;

    // 대각선 움직임에 대해 코스트 다르게 부과
    if (abs(node.pose.x - pose_n.x) == 1 && abs(node.pose.y - pose_n.y))
        return 14;
    else
        return 10;
}

double AStar::ComputeHeuristicCost(Pose pose_n)
{
    return (abs(m_goal.x - pose_n.x) + abs(m_goal.y - pose_n.y) * 10);

    // return std::sqrt((m_goal.x - pose_n.x) * (m_goal.x - pose_n.x) + (m_goal.y - pose_n.y) * (m_goal.y - pose_n.y));
}

void AStar::InitializeNode()
{
    Node* init_node = new Node;

    init_node->pose.x = m_departure.x;
    init_node->pose.y = m_departure.y;

    init_node->g_cost = 0;
    init_node->h_cost = ComputeHeuristicCost(m_departure);
    init_node->f_cost = init_node->g_cost + init_node->h_cost;

    init_node->parent = nullptr;

    m_open_list.push_back(init_node);
}

bool AStar::IsOnList(Node node, std::vector<Node*> list)
{
    for (int i = 0; i < list.size(); i++)
    {
        if (node == *list[i])
            return true;
    }

    return false;
}

bool AStar::IsNodeOnGoal()
{
    // 마지막 노드가 close list 에 있으면 도착했다고 출력

    Node goal_node;
    goal_node.pose.x = m_goal.x;
    goal_node.pose.y = m_goal.y;

    if (IsOnList(goal_node, m_close_list))
        return true;

    return false;
}

void AStar::ReconstructPath(Node* current_node)
{
    // for (int i = 0; i < m_close_list.size(); i++)
    // {
    //     m_map[m_close_list[i]->pose.x][m_close_list[i]->pose.y] = '+';
    // }

    // std::printf("close list size = %ld \n\n", m_close_list.size());

    ////////////////////////////////////////////////
    Node* p = current_node;

    while (p != nullptr)
    {
        m_map[p->pose.x][p->pose.y] = '+';
        p = p->parent;
    }
    ////////////////////////////////////////////////

    // Node* p = current_node;

    // while (p != nullptr)
    // {
    //     m_map[p->pose.x][p->pose.y] = '+';
    //     p = p->parent;
    // }

    // std::printf("return path \n\n");
}

bool AStar::Run()
{
    std::printf("A Star algorithm start\n");

    int current_node_index = 0;
    Node* current_node;

    while (!m_open_list.empty())
    {
        current_node_index = 0;
        current_node = m_open_list[0];

        // cost가 가장 작은 노드를 선택함.
        for (int i = 0; i < m_open_list.size(); i++)
        {
            if (m_open_list[i]->f_cost < current_node->f_cost)
            {
                current_node_index = i;
                current_node = m_open_list[i];
            }
        }

        m_open_list.erase(m_open_list.begin() + current_node_index);
        m_close_list.push_back(current_node);

        for (auto propagation_value : move)
        {
            Node* next_node = new Node;
            Pose next_pose;

            next_pose.x = current_node->pose.x + propagation_value.first;
            next_pose.y = current_node->pose.y + propagation_value.second;

            // 그리드 맵을 벗어나는 위치는 pass 해야함.
            if (next_pose.x < 0 || next_pose.x >= m_width || next_pose.y < 0 || next_pose.y >= m_height)
                continue;

            // 장애물 있는 곳은 pass 해야함.
            else if (m_map[next_pose.x][next_pose.y] == 'o')
                continue;

            else
            {
                next_node->pose.x = next_pose.x;
                next_node->pose.y = next_pose.y;
            }

            // close list 에 있으면 패스하기
            if (IsOnList(*next_node, m_close_list))
                continue;

            // open list에 없으면 openlist에 추가하고 parent 를 현재 노드로 설정.
            else if (!IsOnList(*next_node, m_open_list))
            {
                next_node->parent = current_node;

                next_node->g_cost = current_node->g_cost + ComputeActualCost(*current_node, next_pose);
                next_node->h_cost = ComputeHeuristicCost(next_pose);
                next_node->f_cost = next_node->g_cost + next_node->h_cost;

                m_open_list.push_back(next_node);
            }

            // open list에 있지만 cost 가 더 낮은 방향이 있으면 업데이트
            else
            {
                int index = -1;
                for (int i = 0; i < m_open_list.size(); i++)
                {
                    if (next_node->pose.x == m_open_list[i]->pose.x && next_node->pose.y == m_open_list[i]->pose.y)
                    {
                        index = i;
                        break;
                    }
                }

                double g_cost = current_node->g_cost + ComputeActualCost(*current_node, next_pose);

                if (g_cost < m_open_list[index]->g_cost)
                {
                    m_open_list[index]->parent = current_node;
                    m_open_list[index]->g_cost = g_cost;
                    m_open_list[index]->h_cost = ComputeHeuristicCost(next_pose);
                    m_open_list[index]->f_cost = m_open_list[index]->g_cost + m_open_list[index]->h_cost;
                }
                // 열린 목록에 있는 next_pose 위치의 노드가 가지고 있는 g_cost와
                // current_node.g_cost + 다음 노드까지의 cost 를 합한 g_cost 와 비교
            }

            printf("cur | next : (%d,%d) | (%d,%d) \n", current_node->pose.x, current_node->pose.y, next_pose.x, next_pose.y);
        }

        std::printf("end step \n");

        if (IsNodeOnGoal())
        {
            ReconstructPath(current_node);
            return true;
        }
    }

    return false;
}