#include <math.h>
#include <iostream>
#include <queue>
#include <vector>

using namespace std;

typedef struct node
{                           //노드
    int x, y;               //좌표
    int G, H;               // g(node), h(node)
    pair<int, int> parent;  //역추적에 쓰일 이전 노드
} Node;

struct cmp
{  //우선순위 큐 비교 함수
    bool operator()(const node u, const node v)
    {
        if (u.G + u.H > v.G + v.H)
            return true;  // F는 작은게 위로 오게
        else if (u.G + u.H == v.G + v.H)
        {  // F가 같다면 G가 큰게 위로 오게
            if (u.G < v.G)
                return true;
            else
                return false;
        }
        else
            return false;
    }
};

void print_map(vector<vector<char>> map)
{  // 맵 출력 함수
    for (int i = 0; i < map.size(); i++)
    {
        for (int j = 0; j < map.size(); j++)
            cout << map[i][j] << " ";
        cout << '\n';
    }
}

int Astar(vector<vector<char>> map, pair<int, int> start, pair<int, int> goal)
{
    priority_queue<Node, vector<Node>, cmp> open;  // 우선순위 큐

    bool close[10][10] = {
        0,
    };  // 폐쇄 리스트(리스트&visit)
    vector<Node> close_list;
    Node s_node;  // 시작 노드

    int cx[8] = {0, 1, 0, -1, 1, 1, -1, -1};  // 방향 좌표 ↑→↓←↘↗↙↖
    int cy[8] = {-1, 0, 1, 0, 1, -1, 1, -1};

    // 시작지점 초기화
    s_node.x = start.second;
    s_node.y = start.first;
    s_node.G = 0;
    s_node.H = (abs(goal.second - s_node.x) + abs(goal.first - s_node.y)) * 10;
    s_node.parent = make_pair(-1, -1);  // 시작 노드의 부모 노드는 -1,-1
    open.push(s_node);
    close[s_node.y][s_node.x] = true;  // 폐쇄 노드

    vector<vector<char>> result = map;
    while (open.size())
    {
        int x = open.top().x;  // 우선순위 큐에서 top 정보 추출 // 가장 코스트가 작은 노드 선택
        int y = open.top().y;
        int G = open.top().G;

        close_list.push_back(open.top());
        result[y][x] = '.';
        open.pop();

        if (x == goal.second && y == goal.first)
            break;  // 도착 지점이 나오면 끝

        Node add;
        for (int i = 0; i < 4; i++)
        {  // top 노드에서 상하좌우 4방향으로 탐색(i<8이면 8방향)
            int nextX = x + cx[i];
            int nextY = y + cy[i];

            // grid map 안에 좌표가 존재하는지 확인
            if (nextX >= 0 && nextX < map.size() && nextY >= 0 && nextY < map.size())
            {
                // 해당 위치에 장애물이 존재하는지 확인, close list 에 존재하는지 확인
                if (map[nextY][nextX] != 1 && close[nextY][nextX] == false)
                {
                    add.x = nextX;
                    add.y = nextY;
                    add.G = i < 4 ? G + 10 : G + 14;  // 상하좌우면 10, 대각선이면 14(√200)
                    add.H = (abs(goal.second - add.x) + abs(goal.first - add.y)) * 10;
                    add.parent = make_pair(y, x);  // 기존 top노드를 부모 노드로 설정
                    close[nextY][nextX] = true;
                    // result[nextY][nextX] = 9;
                    open.push(add);  // 우선순위 큐에 삽입
                    // system("cls");
                    printf("\n\n");

                    print_map(result);
                }
            }
        }
    }

    int px = close_list.back().x;
    int py = close_list.back().y;
    while (close_list.size())
    {  // close_list를 역추적해 경로 탐색
        if (px == close_list.back().x && py == close_list.back().y)
        {  // 목표 노드부터 부모 노드를 탐색해 역추적

            if (px == start.first && py == start.second)
                result[py][px] = '@';

            else if (px == goal.first && py == goal.second)
                result[py][px] = 'x';

            else
                result[py][px] = '+';
            px = close_list.back().parent.second;
            py = close_list.back().parent.first;
            // system("cls");
            printf("\n\n");
            print_map(result);
        }
        close_list.pop_back();
    }

    return 0;
}

int main()
{
    int size = 7;
    /*
    cout << "맵 사이즈 : ";
    cin >> size;
    cout << "맵 입력(0:길, 1:벽, 5: 출발점, 6 : 도착점)\n";
    */
    // vector<vector<int>> map(size, vector<int>(size, 0));
    // pair<int, int> start, goal;
    // start = {5, 1};
    // goal = {1, 5};
    // map = {
    //     {0, 0, 0, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 0, 6, 0},
    //     {0, 0, 0, 0, 0, 0, 0},
    //     {0, 1, 1, 1, 1, 0, 0},
    //     {0, 0, 0, 0, 0, 0, 0},
    //     {0, 5, 0, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 0, 0, 0},
    // };

    vector<vector<char>> map(size, vector<char>(size, '.'));
    pair<int, int> start, goal;
    start = {0, 0};
    goal = {6, 6};
    map = {
        {'.', '.', '.', '.', '.', '.', '.'},
        {'.', '.', '.', '.', '.', '.', '.'},
        {'.', '.', '.', '.', '.', '.', '.'},
        {'.', '.', '.', '.', '.', '.', '.'},
        {'.', '.', '.', '.', '.', '.', '.'},
        {'.', '.', '.', '.', '.', '.', '.'},
        {'.', '.', '.', '.', '.', '.', '.'},
    };

    print_map(map);
    Astar(map, start, goal);

    // start = {4, 4};
    // goal = {8, 6};
    // map = {
    //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    //     {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
    //     {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
    //     {0, 0, 0, 0, 5, 0, 0, 1, 0, 0},
    //     {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
    //     {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 1, 0, 6, 0, 0, 0},
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
    // };
    // print_map(map);
    // Astar(map, start, goal);

    return 0;
}
