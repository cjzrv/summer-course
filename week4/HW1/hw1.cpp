#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdio>

#define MAX_ITERATION 10000
#define W_CONT 1        // 測試圖片簡單且邊緣明確，不需太高的 cont 權重
#define W_CURV 3        // curv 權重用於保持輪廓的平滑性
#define W_IMG 10        // 燈泡的邊緣清晰，設較高 img 權重能更貼近邊緣
#define RANGE 8

using namespace std;
using namespace cv;

vector<Point> points;
Mat org_img, temp_img, click_img;

void onMouse(int, int, int, int, void*);
bool ACTIVE_CONTOUR(Mat);
void reshape(Mat);
void defaultPoints(Size);
void splitPoints();

int main()
{
    org_img = imread("./img.jpg");
    if(org_img.empty()) return -1;

    cvtColor(org_img, temp_img, COLOR_BGR2GRAY);
    GaussianBlur(temp_img, temp_img, Size(9, 9), 0);

    Mat tempX, tempY;
    Sobel(temp_img, tempX, CV_64F, 1, 0, 3);
    Sobel(temp_img, tempY, CV_64F, 0, 1, 3);
    magnitude(tempX, tempY, temp_img);

    namedWindow("Active Contour", WINDOW_NORMAL);
    resizeWindow("Active Contour", 800, 870);
    click_img = org_img.clone();
    setMouseCallback("Active Contour", onMouse);
    imshow("Active Contour", click_img);
    waitKey(0);
    if(points.size() < 4)   // 若少於四個點則以圖片的四個頂點及四個邊的中點作為起始點
        defaultPoints(org_img.size());

    int count = 0;
    for(; ACTIVE_CONTOUR(temp_img) && count < MAX_ITERATION; count++)
    {
        reshape(org_img);
        if(count == 10) splitPoints();  // 在每兩點之間插入中間點，使點的數量變為兩倍
        if(count == 30) splitPoints();
        if(count == 40) splitPoints();
    }

    printf("\nIteration: %d\nPoints: %lu\nDONE!!\n", count, points.size());
    waitKey(0);
    destroyAllWindows();
    return 0;
}

bool ACTIVE_CONTOUR(Mat grad_img)
{
    bool continue_fuck = false;

    for(int i = 0; i < points.size(); i++)
    {
        Point previous = points[(i != 0) ? (i - 1) : (points.size()-1)];
        Point next = points[(i + 1) % points.size()];
        Point temp = points[i];
        double e_min = DBL_MAX;

        for(int j = -RANGE; j <= RANGE; j++)
        {
            for(int k = -RANGE; k <= RANGE; k++)
            {
                Point current(points[i].x + j, points[i].y + k);

                if(current.x < 0 
                 || current.x >= grad_img.cols
                 || current.y < 0
                 || current.y >= grad_img.rows)
                    continue;

                double e_cont = norm(current - points[i]);
                double e_curv = norm(previous - 2 * current + next);
                double e_img = -abs(grad_img.at<double>(current));  // 是負號，我加了負號
                double e_total = W_CONT * e_cont + W_CURV * e_curv + W_IMG * e_img;

                if(e_total < e_min)
                {
                    e_min = e_total;
                    temp = current;
                }
            }
        }
        if(points[i] != temp)
        {
            points[i] = temp;
            continue_fuck = true;
        }
    }
    return continue_fuck;
}

void onMouse(int event, int x, int y, int flag, void* param)
{
    if(event == EVENT_LBUTTONDOWN)
    {
        points.push_back(Point(x,y));
        circle(click_img, Point(x,y), 2, CV_RGB(204, 0, 0), 5);
        imshow("Active Contour", click_img);
    }
}

void reshape(Mat img)
{
    Mat contour_img = img.clone();
    for(int i = 0; i < points.size(); i++)
    {
        int next = (i + 1) % points.size();
        circle(contour_img, points[i], 2, CV_RGB(204, 0, 0), 2);
        line(contour_img, points[i], points[next], CV_RGB(204, 0, 0), 2);
    }
    imshow("Active Contour", contour_img);
    waitKey(30);
}

void defaultPoints(Size imgSize)
{
    int width = imgSize.width;
    int height = imgSize.height;
    points.clear();
    points.push_back(Point(0, 0));
    points.push_back(Point(width / 2, 0));
    points.push_back(Point(width - 1, 0));
    points.push_back(Point(width - 1, height / 2));
    points.push_back(Point(width - 1, height - 1));
    points.push_back(Point(width / 2, height - 1));
    points.push_back(Point(0, height - 1));
    points.push_back(Point(0, height / 2));
}

void splitPoints()
{
    vector<Point> tempPoints;
    for(int i = 0; i < points.size(); i++)
    {
        Point p1 = points[i];
        Point p2 = points[(i + 1) % points.size()];
        tempPoints.push_back(p1);
        Point midpoint((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
        tempPoints.push_back(midpoint);
    }
    points = tempPoints;
}