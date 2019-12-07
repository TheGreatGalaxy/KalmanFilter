#include <iostream>
#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
// #include <climit>
#include "Eigen/Dense"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "ukf_cv.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


void check_arguments(int argc, char* argv[]) {
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";

    bool has_valid_args = false;

    // make sure the user has provided input and output files
    if (argc == 1) {
        cerr << usage_instructions << endl;
    } else if (argc == 2) {
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    } else if (argc == 3) {
        has_valid_args = true;
    } else if (argc > 3) {
        cerr << "Too many arguments.\n" << usage_instructions << endl;
    }

    if (!has_valid_args) {
        exit(EXIT_FAILURE);
    }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!out_file.is_open()) {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}

Eigen::VectorXd CalculateRMSE(const vector<Eigen::VectorXd> &estimations,
                       const vector<Eigen::VectorXd> &ground_truth) {
    Eigen::VectorXd rmse(4);
    rmse << 0,0,0,0;
    double bias_x_max = -3000000000.0;
    double bias_x_min = 3000000000.0;
    double bias_y_max = -3000000000.0;
    double bias_y_min = 3000000000.0;
    double bias_vx_max = -3000000000.0;
    double bias_vx_min = 3000000000.0; 
    double bias_vy_max = -3000000000.0;
    double bias_vy_min = 3000000000.0; 


    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
        cout<<"the input is not legal!!!"<<endl;
        return rmse;
    }

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        Eigen::VectorXd residual = estimations[i] - ground_truth[i];
        bias_x_max = residual[0]>bias_x_max?residual[0]:bias_x_max;
        bias_x_min = residual[0]<bias_x_min?residual[0]:bias_x_min;
        bias_y_max = residual[1]>bias_y_max?residual[1]:bias_y_max;
        bias_y_min = residual[1]<bias_y_min?residual[1]:bias_y_min;
        bias_vx_max = residual[2]>bias_vx_max?residual[2]:bias_vx_max;
        bias_vx_min = residual[2]<bias_vx_min?residual[2]:bias_vx_min;
        bias_vy_max = residual[3]>bias_vy_max?residual[3]:bias_vy_max;
        bias_vy_min = residual[3]<bias_vy_min?residual[3]:bias_vy_min;

        residual = residual.array() * residual.array();
        rmse = rmse + residual;
    }
    //cout 最大、最小偏差
    std::cout<<"bias_x_max: "<<bias_x_max<<std::endl<<
            "bias_x_min: "<<bias_x_min<<std::endl<<
            "bias_y_max: "<<bias_y_max<<std::endl<<
            "bias_y_min: "<<bias_y_min<<std::endl<<
            "bias_vx_max: "<<bias_vx_max<<std::endl<<
            "bias_vx_min: "<<bias_vx_min<<std::endl<<
            "bias_vy_max: "<<bias_vy_max<<std::endl<<
            "bias_vy_min: "<<bias_vy_min<<std::endl;
    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;

}
double mean_filter(vector<double> &vec, double data){
    int len = vec.size();
    if(len>4)
        vec.erase(vec.begin());
    vec.push_back(data);
    double avg = 0;
    for(auto &d:vec)
        avg += d/len;
    return avg;
}


int main(int argc, char* argv[]) {
    // check_arguments(argc, argv);

    // string in_file_name_ = argv[1];
    string in_file_name_ = "/home/guangtong/project/ukf-code/data/data_synthetic.txt";
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    // string out_file_name_ = argv[2];
    string out_file_name_= "/home/guangtong/project/ukf-code/data/output1.txt";
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    check_files(in_file_, in_file_name_, out_file_, out_file_name_);

    vector<MeasurementPackage> measurement_pack_list;
    vector<GroundTruthPackage> gt_pack_list;

    string line;

    // prep the measurement packages (each line represents a measurement at a
    // timestamp)
    while (getline(in_file_, line)) {

        string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        istringstream iss(line);
        long long timestamp;

        // reads first element from the current line
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0) {
            // LASER MEASUREMENT

            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = Eigen::VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        } else if (sensor_type.compare("R") == 0) {
            // RADAR MEASUREMENT
            continue;
            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = Eigen::VectorXd(3);
            float ro;
            float phi;
            float ro_dot;
            iss >> ro;
            iss >> phi;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, phi, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }

        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = Eigen::VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        gt_pack_list.push_back(gt_package);
    }

    // Create a Fusion EKF instance
    UKF ukf;
    // used to compute the RMSE later
    vector<Eigen::VectorXd> estimations;
    vector<Eigen::VectorXd> ground_truth;
    vector<double> vx_vec;
    vector<double> vy_vec;

    //Call the EKF-based fusion
    size_t N = measurement_pack_list.size();
    for (size_t k = 0; k < N; ++k) {
        // start filtering from the second frame (the speed is unknown in the first
        // frame)
        ukf.ProcessMeasurement(measurement_pack_list[k]);
        double p_x = ukf.x_(0);
        double p_y = ukf.x_(1);
        double vx = ukf.x_(2);
        double vy = ukf.x_(3);

        // double v1 = cos(yaw) * v;
        // double v2 = sin(yaw) * v;

        Eigen::VectorXd estimate(4);

        estimate(0) = p_x;
        estimate(1) = p_y;
        estimate(2) = vx;
        estimate(3) = vy;

        // output the estimation
        out_file_ << p_x << " ";
        out_file_ << p_y << " ";
        out_file_ << vx << " ";
        out_file_ << vy << " ";
        out_file_ << mean_filter(vx_vec,vx)<<" ";
        out_file_ << mean_filter(vy_vec,vy)<<" ";
        double yaw = std::atan2(vy,vx);
        if(yaw<-1)
            yaw = yaw + 2*M_PI;
        out_file_ << yaw <<" ";
        // output the measurements
        if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
            // output the estimation
            out_file_ << measurement_pack_list[k].raw_measurements_(0) << " ";
            out_file_ << measurement_pack_list[k].raw_measurements_(1) << " ";
        } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
            // output the estimation in the cartesian coordinates
            float ro = measurement_pack_list[k].raw_measurements_(0);
            float phi = measurement_pack_list[k].raw_measurements_(1);
            out_file_ << ro * cos(phi) << " "; // p1_meas
            out_file_ << ro * sin(phi) << " "; // ps_meas
        }

        // output the ground truth packages
        out_file_ << gt_pack_list[k].gt_values_(0) << " ";
        out_file_ << gt_pack_list[k].gt_values_(1) << " ";
        out_file_ << gt_pack_list[k].gt_values_(2) << " ";
        out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

        estimations.push_back(estimate);
        ground_truth.push_back(gt_pack_list[k].gt_values_);
    }

    // compute the accuracy (RMSE)
    cout << "Accuracy - RMSE:" << endl << CalculateRMSE(estimations, ground_truth) << endl;

    // close files
    if (out_file_.is_open()) {
        out_file_.close();
    }

    if (in_file_.is_open()) {
        in_file_.close();
    }

    return 0;
}