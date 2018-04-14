#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <zconf.h>

using namespace std;

// path to trajectory file
string ground_truth_trajectory_file = "./groundtruth.txt";
string estimate_trajectory_file = "./estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> generate_pose(string file_path);

int main(int argc, char **argv) {
    auto ground_truth_pose = generate_pose(ground_truth_trajectory_file);
    auto estimate_pose = generate_pose(estimate_trajectory_file);

    int n = estimate_pose.size();
    double RMSE = 0.0;
    for(int i = 0; i < n; i++) {
        auto tgi = ground_truth_pose[i];
        auto tei = estimate_pose[i];
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        Vector6d tmp = (tgi.inverse() * tei).log();
        double ei = tmp.transpose() * tmp;
        RMSE += ei;
    }

    cout << "RMSE:" << sqrt(RMSE / n) << endl;
    DrawTrajectory(ground_truth_pose, estimate_pose);
};

vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> generate_pose(string trajectory_file) {
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your code here (5~10 lines)
    // read in the trajectory file, add convert it to poses
    ifstream inFile;
    inFile.open(trajectory_file);
    string line;
    while(inFile.good() && (getline(inFile, line))) {
        istringstream iss(line);
        double t, tx, ty, tz, qx, qy, qz, qw;
        iss >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Quaterniond q = Eigen::Quaterniond(qx, qy, qz, qw);
        Eigen::Vector3d v = Eigen::Vector3d(tx, ty, tz);
        Sophus::SE3 se3 = Sophus::SE3(q, v);
        poses.push_back(se3);

    }
    return poses;
};

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
    if (poses1.empty() || poses2.empty()) {
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
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            glColor3f(1 - (float) i / poses1.size(), 0.0f, (float) i / poses1.size());
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        //usleep(5000);   // sleep 5 ms
    }

}