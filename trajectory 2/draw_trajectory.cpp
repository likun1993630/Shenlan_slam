#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "./trajectory.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

    // 定义 vector 顺序容器，容器内对象为李群SE3，容器名称为poses
    //此处容器内元素不是C++中的标准类型如int，所以要强调内存分配
    //vector有两个参数，后面的参数一般是默认的，这里用适合Eigen库的对齐方式来初始化容器
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your code here (5~10 lines)

    //打开文件
    ifstream infile;
    infile.open(trajectory_file, ios::in);

    //判断文件是否被打开
    if (!infile.is_open())
        cout << "Open file failure" << endl;

    // 读取文件
    string line;
    /*
     * infile.eof() 读到文件最后一个字符时返回false
     * getline(infile, line)检查流是否有错误，如果数据流没有错误并且已准备好进行 I/O操作，则返回true，
     * 如果到了infile的最后，返回false
     * getline（读出的目标位置，被存放的目标位置）
     * 使用stringstream对象简化类型转换 ，安全和自动的类型转换。
     */

        while (!infile.eof() && getline(infile, line)) {
            stringstream ss(line); //string类型的line赋值给字符串流ss，空格换行指标都会被当做切分符,相当于stringstream ss;ss<<line;
            double str;
            vector<double> arr;
            while (ss >> str) {
                cout << str << " ";
                arr.push_back(str);
            }
            Eigen::Quaterniond q(arr[7], arr[4], arr[5], arr[6]);
            Eigen::Vector3d t(arr[1], arr[2], arr[3]);
            Sophus::SE3 SE3(q, t);
            poses.push_back(SE3);
            cout << endl;
            cout << line << endl;
        }

    //关闭txt文件
    infile.close();
    //// end your code here

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}