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
    //点云地图传入方式

    //将文件从硬盘读取到内存，并命名为posefile
    ifstream posefile;
    posefile.open(trajectory_file, ios::in);  //ios::in 为输入(读)而打开文件
    //ifstream posefile(trajectory_file);

    //判断文件是否正确打开
    if (!posefile) {
        cout << "Open file failure." << endl;
        return 1;
    }

    //使用for循环读入数据
    ///基于范围的for循环，表示从data数组的第一项开始 循环遍历  auto表示自动根据后面的元素 获得符合要求的类型
    for (int i = 0; i<620; i++) {
        double data[8] = { 0 }; //初始化一个double类型
        for (auto& d : data) //auto自动类型转换
            posefile >> d; //文件流类型的变量posefile将trajectory.txt中的数据给了d数组
        Eigen::Quaterniond q(data[7], data[4], data[5], data[6]);
        Eigen::Vector3d t(data[1], data[2], data[3]);
        Sophus::SE3 SE3_traj(q, t);
        poses.push_back(SE3_traj);//存储SE（3）到位姿数组中
    }

    posefile.close();
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