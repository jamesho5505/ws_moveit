#include <rclcpp/rclcpp.hpp>
#include <tm_msgs/srv/set_positions.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <vector>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <functional>


// ================= Butterworth Filter Class =================
class ButterworthFilter
{
public:
    ButterworthFilter() = default;

    void setup(int order, double cutoff_hz, double sample_rate_hz)
    {
        order_ = order;
        double wc = std::tan(M_PI * cutoff_hz / sample_rate_hz);
        double k1 = 1.414213562; // sqrt(2)

        if (order == 1)
        {
            double k = wc;
            a_coeffs_ = {1.0, (k - 1) / (k + 1)};
            b_coeffs_ = {k / (k + 1), k / (k + 1)};
        }
        else // 2nd order
        {
            double k = wc / k1;
            double k_sq = k * k;
            double den = k_sq + k1*k + 1;
            a_coeffs_ = {1.0, 2 * (k_sq - 1) / den, (k_sq - k1*k + 1) / den};
            b_coeffs_ = {k_sq / den, 2 * k_sq / den, k_sq / den};
        }

        x_hist_.assign(order_ + 1, 0.0);
        y_hist_.assign(order_ + 1, 0.0);
    }

    double filter(double input)
    {
        // Shift history
        for (int i = order_; i > 0; --i) {
            x_hist_[i] = x_hist_[i-1];
            y_hist_[i] = y_hist_[i-1];
        }
        x_hist_[0] = input;

        double output = 0.0;
        for (int i = 0; i <= order_; ++i) {
            output += b_coeffs_[i] * x_hist_[i];
            if (i > 0) output -= a_coeffs_[i] * y_hist_[i];
        }
        y_hist_[0] = output;
        return output;
    }

private:
    int order_{2};
    std::vector<double> a_coeffs_, b_coeffs_;
    std::vector<double> x_hist_, y_hist_;
};

// ================= Lead Filter Class =================
class LeadFilter
{
public:
    LeadFilter() = default;

    void setup(double lead_time_constant,
               double alpha,
               double sample_rate_hz)
    {
        double sample_period = 1.0 / sample_rate_hz;
        double K = 2.0 / sample_period;

        double denominator = 1.0 + K * alpha * lead_time_constant;

        coefficient_a0_ = (1.0 + K * lead_time_constant) / denominator;
        coefficient_a1_ = (1.0 - K * lead_time_constant) / denominator;
        coefficient_b1_ = (1.0 - K * alpha * lead_time_constant) / denominator;

        previous_input_  = 0.0;
        previous_output_ = 0.0;
    }

    double filter(double current_input)
    {
        double current_output =
            coefficient_a0_ * current_input +
            coefficient_a1_ * previous_input_ -
            coefficient_b1_ * previous_output_;

        previous_input_  = current_input;
        previous_output_ = current_output;

        return current_output;
    }

private:
    double coefficient_a0_{0.0};
    double coefficient_a1_{0.0};
    double coefficient_b1_{0.0};

    double previous_input_{0.0};
    double previous_output_{0.0};
};


class FtCompNode : public rclcpp::Node
{
public:
    FtCompNode()
    : Node("tm_ft_compensation_node")
    {
        // 互動式校正設定
        data_number_       = declare_parameter<int>("data_number", 9);       // 姿態數量
        duration_per_pose_ = declare_parameter<double>("duration_per_pose", 5.0); // 每個姿態 CSV 蒐集時間 (s)
        sample_rate_hz_    = declare_parameter<double>("sample_rate", 100.0);      // CSV 取樣率
        tool_mass_guess_   = declare_parameter<double>("tool_mass_guess", 1.0547); // 舊程式裡用來算 Tool_Weight_Matrix 的 mass

        pose_topic_   = declare_parameter<std::string>("pose_topic", "/tool_pose");
        wrench_topic_ = declare_parameter<std::string>("wrench_topic", "/robotiq_force_torque_sensor_broadcaster/wrench");
        output_topic_ = declare_parameter<std::string>("output_topic", "/ft_compensated");

        // 濾波器參數
        int    filter_order   = declare_parameter<int>("filter_order", 2);
        double filter_cutoff  = declare_parameter<double>("filter_cutoff_hz", 10.0);
        // double lead_time_constant = declare_parameter<double>("lead_T", 0.03);
        // double lead_alpha         = declare_parameter<double>("lead_alpha", 0.2);

        // subscribers / publisher
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10,
            std::bind(&FtCompNode::poseCallback, this, std::placeholders::_1));

        wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            wrench_topic_, 100,
            std::bind(&FtCompNode::wrenchCallback, this, std::placeholders::_1));

        wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
            output_topic_, 100);

        move_client_ = create_client<tm_msgs::srv::SetPositions>("set_positions");

        // 初始化 6 個濾波器
        for(int i=0; i<6; ++i) {
            filters_.emplace_back();
            filters_.back().setup(filter_order, filter_cutoff, sample_rate_hz_);
        }
        // for (int i = 0; i < 6; ++i) {
        //     filters_.emplace_back();
        //     filters_.back().setup(
        //         lead_time_constant,
        //         lead_alpha,
        //         sample_rate_hz_);
        // }

        RCLCPP_INFO(get_logger(), "ft_compensation_node started.");
        RCLCPP_INFO(get_logger(), "data_number = %d, duration_per_pose = %.1f s, sample_rate = %.1f Hz",
                    data_number_, duration_per_pose_, sample_rate_hz_);
        RCLCPP_INFO(get_logger(), "Butterworth filter: order=%d, cutoff=%.1f Hz",
                    filter_order, filter_cutoff);
        if (filter_order < 1 || filter_order > 2) {
            RCLCPP_WARN(get_logger(), "Filter order %d is not supported, using order 2.", filter_order);
        }

    }

    void run()
    {
        // 等待至少有一筆 wrench 與 pose
        waitForFirstData();
        

        // 1) 初始偏值估計 (與舊程式相同邏輯)
        double fx_initial = 0.0, fy_initial = 0.0, fz_initial = 0.0;
        double mx_initial = 0.0, my_initial = 0.0, mz_initial = 0.0;
        estimateInitialBias(fx_initial, fy_initial, fz_initial,
                            mx_initial, my_initial, mz_initial);

        // 2) 資料結構準備
        const int ROW = 3 * data_number_;
        const int COL = 6;

        // 動態矩陣配置函式
        auto allocateMatrix = [](int row, int col) {
            double **M = new double *[row];
            for (int i = 0; i < row; ++i) {
                M[i] = new double[col]{0.0};
            }
            return M;
        };

        // auto printMatrix = [](double **M, int row, int col) {
        //     for (int i = 0; i < row; ++i) {
        //         for (int j = 0; j < col; ++j) {
        //             std::cout << M[i][j] << " ";
        //         }
        //         std::cout << std::endl;
        //     }
        // };

        auto trasposeMatrix = [&](double **M, double **MT, int row, int col) {
            for (int i = 0; i < col; ++i) {
                for (int j = 0; j < row; ++j) {
                    MT[i][j] = M[j][i];
                }
            }
        };

        // determinant, cofactor, inverse 與原 C++ 一樣
        std::function<double(double **, int)> determinantMatrix =
            [&](double **A, int n) -> double
        {
            if (n == 1) {
                return A[0][0];
            }
            double det = 0.0;
            double **temp = allocateMatrix(n, n);
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n - 1; ++j) {
                    for (int k = 0; k < n - 1; ++k) {
                        temp[j][k] = A[j + 1][(k >= i) ? k + 1 : k];
                    }
                }
                double t = determinantMatrix(temp, n - 1);
                if (i % 2 == 0) {
                    det += A[0][i] * t;
                } else {
                    det -= A[0][i] * t;
                }
            }
            return det;
        };

        auto getCofactor = [&](double **A, int n, double **C) {
            if (n == 1) {
                C[0][0] = 1.0;
                return;
            }
            double **temp = allocateMatrix(n, n);
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    for (int k = 0; k < n - 1; ++k) {
                        for (int t = 0; t < n - 1; ++t) {
                            temp[k][t] = A[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
                        }
                    }
                    C[j][i] = determinantMatrix(temp, n - 1); // 同時轉置
                    if ((i + j) % 2 == 1) {
                        C[j][i] = -C[j][i];
                    }
                }
            }
        };

        auto inverseMatrix = [&](double **A, int n, double **Ainv) -> bool {
            double det = determinantMatrix(A, n);
            if (det == 0.0) {
                std::cout << "Determinant Matrix Was 0" << std::endl;
                return false;
            }
            double **C = allocateMatrix(n, n);
            getCofactor(A, n, C);
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    Ainv[i][j] = C[i][j] / det;
                }
            }
            return true;
        };

        auto multiplyMatrix = [&](double **L, int r1, int c1,
                                  double **R, int r2, int c2,
                                  double **Out)
        {
            if (c1 != r2) return;
            for (int i = 0; i < r1; ++i) {
                for (int j = 0; j < c2; ++j) {
                    Out[i][j] = 0.0;
                    for (int k = 0; k < c1; ++k) {
                        Out[i][j] += L[i][k] * R[k][j];
                    }
                }
            }
        };

        // 主要矩陣
        double **Force_Matrix                = allocateMatrix(ROW, COL);
        double **Torque_Matrix               = allocateMatrix(ROW, 1);
        double **Force_Matrix_k              = allocateMatrix(ROW, 1);
        double **Rotation_Matrix_k           = allocateMatrix(3, 3);
        double **Transposed_Rotation_Matrix_k= allocateMatrix(3, 3);
        double **Rotation_Matrix_k_calculate = allocateMatrix(ROW, COL);
        double **GravityXYZ_Matrix_erro      = allocateMatrix(3, 1);
        double **Tool_Weight_Matrix          = allocateMatrix(3, 1);

        // 舊程式中用來估計誤差的 Tool_Weight_Matrix（初步）
        Tool_Weight_Matrix[0][0] = 0.0;
        Tool_Weight_Matrix[1][0] = 0.0;
        Tool_Weight_Matrix[2][0] = -1.0 * tool_mass_guess_ * 9.787;

        double Force_x_erro = 0.0;
        double Force_y_erro = 0.0;
        double Force_z_erro = 0.0;

        // CSV 設定
        const std::string csv_prefix  = "R";
        const std::string mat_suffix  = "_matrix.csv";

        // === 3) 互動式收集 9 姿態（與原 C++ 行為相同，但額外寫 CSV） ===

        for (int i = 0; i < data_number_; ++i)
        {
            int pose_index = i + 1;
            int row = i * 3;
            RCLCPP_INFO(get_logger(), "--- Automatically Moving to Pose %d/%d ---", i + 1, data_number_);

            // 1. 命令機器人移動
            if (!sendRobotMove(calibration_poses[i])) {
                RCLCPP_ERROR(get_logger(), "Failed to reach pose %d. Aborting.", i + 1);
                return;
            }

            // 2. 等待機器人完全穩定 (沿用您現有的 waitForPoseStabilized)
            waitForPoseStabilized();

            // 3. 獲取當前數據 (原本的採集邏輯)
            rclcpp::spin_some(shared_from_this());


            double fx = fx_;
            double fy = fy_;
            double fz = fz_;
            double mx = mx_;
            double my = my_;
            double mz = mz_;

            // 填入 Force_Matrix / Torque_Matrix（照原 C++）
            Force_Matrix_k[row][0]     = fx;
            Force_Matrix_k[row + 1][0] = fy;
            Force_Matrix_k[row + 2][0] = fz;

            Force_Matrix[row][0]     = 0;
            Force_Matrix[row][1]     = fz;
            Force_Matrix[row][2]     = -fy;
            Force_Matrix[row][3]     = 1;
            Force_Matrix[row][4]     = 0;
            Force_Matrix[row][5]     = 0;

            Force_Matrix[row + 1][0] = -fz;
            Force_Matrix[row + 1][1] = 0;
            Force_Matrix[row + 1][2] = fx;
            Force_Matrix[row + 1][3] = 0;
            Force_Matrix[row + 1][4] = 1;
            Force_Matrix[row + 1][5] = 0;

            Force_Matrix[row + 2][0] = fy;
            Force_Matrix[row + 2][1] = -fx;
            Force_Matrix[row + 2][2] = 0;
            Force_Matrix[row + 2][3] = 0;
            Force_Matrix[row + 2][4] = 0;
            Force_Matrix[row + 2][5] = 1;

            Torque_Matrix[row][0]     = mx;
            Torque_Matrix[row + 1][0] = my;
            Torque_Matrix[row + 2][0] = mz;

            // 以當前 pose 姿態算 rotation matrix
            tf2::Quaternion q(
                current_pose_.pose.orientation.x,
                current_pose_.pose.orientation.y,
                current_pose_.pose.orientation.z,
                current_pose_.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double rx, ry, rz;
            m.getRPY(rx, ry, rz);

            Rotation_Matrix_k[0][0] = std::cos(rz) * std::cos(ry);
            Rotation_Matrix_k[0][1] = -std::sin(rz) * std::cos(rx) + std::cos(rz) * std::sin(ry) * std::sin(rx);
            Rotation_Matrix_k[0][2] =  std::sin(rz) * std::sin(rx) + std::cos(rz) * std::sin(ry) * std::cos(rx);
            Rotation_Matrix_k[1][0] = std::sin(rz) * std::cos(ry);
            Rotation_Matrix_k[1][1] =  std::cos(rz) * std::cos(rx) + std::sin(rz) * std::sin(ry) * std::sin(rx);
            Rotation_Matrix_k[1][2] = -std::cos(rz) * std::sin(rx) + std::sin(rz) * std::sin(ry) * std::cos(rx);
            Rotation_Matrix_k[2][0] = -std::sin(ry);
            Rotation_Matrix_k[2][1] =  std::cos(ry) * std::sin(rx);
            Rotation_Matrix_k[2][2] =  std::cos(ry) * std::cos(rx);

            // R^T * Tool_Weight_Matrix → GravityXYZ_Matrix_erro
            trasposeMatrix(Rotation_Matrix_k, Transposed_Rotation_Matrix_k, 3, 3);
            multiplyMatrix(Transposed_Rotation_Matrix_k, 3, 3,
                           Tool_Weight_Matrix, 3, 1,
                           GravityXYZ_Matrix_erro);

            Force_x_erro += fx - GravityXYZ_Matrix_erro[0][0];
            Force_y_erro += fy - GravityXYZ_Matrix_erro[1][0];
            Force_z_erro += fz - GravityXYZ_Matrix_erro[2][0];

            // 填入 Rotation_Matrix_k_calculate（照原 C++）
            Rotation_Matrix_k_calculate[row][0]     = Transposed_Rotation_Matrix_k[0][0];
            Rotation_Matrix_k_calculate[row][1]     = Transposed_Rotation_Matrix_k[0][1];
            Rotation_Matrix_k_calculate[row][2]     = Transposed_Rotation_Matrix_k[0][2];
            Rotation_Matrix_k_calculate[row][3]     = 1;
            Rotation_Matrix_k_calculate[row][4]     = 0;
            Rotation_Matrix_k_calculate[row][5]     = 0;

            Rotation_Matrix_k_calculate[row + 1][0] = Transposed_Rotation_Matrix_k[1][0];
            Rotation_Matrix_k_calculate[row + 1][1] = Transposed_Rotation_Matrix_k[1][1];
            Rotation_Matrix_k_calculate[row + 1][2] = Transposed_Rotation_Matrix_k[1][2];
            Rotation_Matrix_k_calculate[row + 1][3] = 0;
            Rotation_Matrix_k_calculate[row + 1][4] = 1;
            Rotation_Matrix_k_calculate[row + 1][5] = 0;

            Rotation_Matrix_k_calculate[row + 2][0] = Transposed_Rotation_Matrix_k[2][0];
            Rotation_Matrix_k_calculate[row + 2][1] = Transposed_Rotation_Matrix_k[2][1];
            Rotation_Matrix_k_calculate[row + 2][2] = Transposed_Rotation_Matrix_k[2][2];
            Rotation_Matrix_k_calculate[row + 2][3] = 0;
            Rotation_Matrix_k_calculate[row + 2][4] = 0;
            Rotation_Matrix_k_calculate[row + 2][5] = 1;

            RCLCPP_INFO(get_logger(),
                        "Pose %d: Rx=%.2f, Ry=%.2f, Rz=%.2f deg, Fx=%.3f, Fy=%.3f, Fz=%.3f, Mx=%.3f, My=%.3f, Mz=%.3f",
                        pose_index,
                        rx / (2 * M_PI) * 360.0,
                        ry / (2 * M_PI) * 360.0,
                        rz / (2 * M_PI) * 360.0,
                        fx, fy, fz, mx, my, mz);

            // ===== CSV 蒐集 (不影響校正公式，只是額外紀錄) =====
            std::string csv_name  = csv_prefix + std::to_string(pose_index) + ".csv";
            std::string mat_name  = csv_prefix + std::to_string(pose_index) + mat_suffix;

            std::ofstream csv(csv_name);
            if (!csv.is_open()) {
                RCLCPP_ERROR(get_logger(), "Failed to open %s for writing.", csv_name.c_str());
            } else {
                csv << "t,Fx,Fy,Fz,Tx,Ty,Tz\n";
                // double dt = 1.0 / sample_rate_hz_;
                int total_steps = static_cast<int>(duration_per_pose_ * sample_rate_hz_);
                rclcpp::Rate rate(sample_rate_hz_);

                auto t0 = now();
                for (int s = 0; s < total_steps && rclcpp::ok(); ++s) {
                    rclcpp::spin_some(this->get_node_base_interface());

                    auto t_now = now();
                    double t_rel = (t_now.seconds() + t_now.nanoseconds()*1e-9) -
                                   (t0.seconds() + t0.nanoseconds()*1e-9);

                    double fx_c = fx_;
                    double fy_c = fy_;
                    double fz_c = fz_;
                    double mx_c = mx_;
                    double my_c = my_;
                    double mz_c = mz_;

                    csv << std::fixed << std::setprecision(6)
                        << t_rel << ","
                        << fx_c << "," << fy_c << "," << fz_c << ","
                        << mx_c << "," << my_c << "," << mz_c << "\n";

                    rate.sleep();
                }
                csv.close();
                RCLCPP_INFO(get_logger(), "Saved CSV for pose %d: %s", pose_index, csv_name.c_str());
            }

            // Rotation matrix 存成 3x3 CSV（B1）
            std::ofstream Rcsv(mat_name);
            if (!Rcsv.is_open()) {
                RCLCPP_ERROR(get_logger(), "Failed to open %s for writing.", mat_name.c_str());
            } else {
                for (int r = 0; r < 3; ++r) {
                    Rcsv << std::fixed << std::setprecision(9)
                         << Rotation_Matrix_k[r][0] << ","
                         << Rotation_Matrix_k[r][1] << ","
                         << Rotation_Matrix_k[r][2];
                    if (r < 2) Rcsv << "\n";
                }
                Rcsv.close();
                RCLCPP_INFO(get_logger(), "Saved rotation matrix for pose %d: %s",
                            pose_index, mat_name.c_str());
            }
        } // end for poses

        // 誤差平均
        Force_x_erro /= static_cast<double>(data_number_);
        Force_y_erro /= static_cast<double>(data_number_);
        Force_z_erro /= static_cast<double>(data_number_);

        // === 4) 照原 C++ 做 COM 與 k1,k2,k3 的解 ===
        double **Transpoed_FT_Matrix      = allocateMatrix(COL, ROW);
        double **Multiplied_Matrix        = allocateMatrix(COL, COL);
        double **Iverse_Matrix            = allocateMatrix(COL, COL);
        double **Multiplied_Matrix_2      = allocateMatrix(COL, ROW);
        double **Multiplied_Matrix_3      = allocateMatrix(COL, 1);

        trasposeMatrix(Force_Matrix, Transpoed_FT_Matrix, ROW, COL);
        multiplyMatrix(Transpoed_FT_Matrix, COL, ROW, Force_Matrix, ROW, COL, Multiplied_Matrix);
        inverseMatrix(Multiplied_Matrix, COL, Iverse_Matrix);
        multiplyMatrix(Iverse_Matrix, COL, COL, Transpoed_FT_Matrix, COL, ROW, Multiplied_Matrix_2);
        multiplyMatrix(Multiplied_Matrix_2, COL, ROW, Torque_Matrix, ROW, 1, Multiplied_Matrix_3);

        double Gravity_center_x = Multiplied_Matrix_3[0][0];
        double Gravity_center_y = Multiplied_Matrix_3[1][0];
        double Gravity_center_z = Multiplied_Matrix_3[2][0];
        double Sensor_Erro_k1   = Multiplied_Matrix_3[3][0];
        double Sensor_Erro_k2   = Multiplied_Matrix_3[4][0];
        double Sensor_Erro_k3   = Multiplied_Matrix_3[5][0];

        RCLCPP_INFO(get_logger(), "Gravity_center = (%.6f, %.6f, %.6f)",
                    Gravity_center_x, Gravity_center_y, Gravity_center_z);
        RCLCPP_INFO(get_logger(), "Sensor_Erro_k = (%.6f, %.6f, %.6f)",
                    Sensor_Erro_k1, Sensor_Erro_k2, Sensor_Erro_k3);

        // 解 Tool weight 與 force 偏置 fx_0,fy_0,fz_0
        double **Transpoed_Rotation_Matrix_k_calculate = allocateMatrix(COL, ROW);
        trasposeMatrix(Rotation_Matrix_k_calculate, Transpoed_Rotation_Matrix_k_calculate, ROW, COL);

        double **Multiplied_Rotation_Matrix    = allocateMatrix(COL, COL);
        double **Iverse_Rotation_Matrix        = allocateMatrix(COL, COL);
        double **Multiplied_Rotation_Matrix_2  = allocateMatrix(COL, ROW);
        double **Multiplied_Rotation_Matrix_3  = allocateMatrix(COL, 1);

        multiplyMatrix(Transpoed_Rotation_Matrix_k_calculate, COL, ROW,
                       Rotation_Matrix_k_calculate, ROW, COL,
                       Multiplied_Rotation_Matrix);
        inverseMatrix(Multiplied_Rotation_Matrix, COL, Iverse_Rotation_Matrix);
        multiplyMatrix(Iverse_Rotation_Matrix, COL, COL,
                       Transpoed_Rotation_Matrix_k_calculate, COL, ROW,
                       Multiplied_Rotation_Matrix_2);
        multiplyMatrix(Multiplied_Rotation_Matrix_2, COL, ROW,
                       Force_Matrix_k, ROW, 1,
                       Multiplied_Rotation_Matrix_3);

        double tw_x = Multiplied_Rotation_Matrix_3[0][0];
        double tw_y = Multiplied_Rotation_Matrix_3[1][0];
        double tw_z = Multiplied_Rotation_Matrix_3[2][0];
        double fx_0 = Multiplied_Rotation_Matrix_3[3][0];
        double fy_0 = Multiplied_Rotation_Matrix_3[4][0];
        double fz_0 = Multiplied_Rotation_Matrix_3[5][0];

        RCLCPP_INFO(get_logger(), "Tool_Weight = (%.6f, %.6f, %.6f)", tw_x, tw_y, tw_z);
        RCLCPP_INFO(get_logger(), "fx_0 = %.6f, fy_0 = %.6f, fz_0 = %.6f", fx_0, fy_0, fz_0);

        double mx_0 = Sensor_Erro_k1 - fy_0 * Gravity_center_z + fz_0 * Gravity_center_y;
        double my_0 = Sensor_Erro_k2 - fz_0 * Gravity_center_x + fx_0 * Gravity_center_z;
        double mz_0 = Sensor_Erro_k3 - fx_0 * Gravity_center_y + fy_0 * Gravity_center_x;

        RCLCPP_INFO(get_logger(), "mx_0 = %.6f, my_0 = %.6f, mz_0 = %.6f",
                    mx_0, my_0, mz_0);

        // === (New) RMSE Calculation Step ===
        RCLCPP_INFO(get_logger(), "=== Calculating RMSE of the calibration model... ===");
        double total_squared_error_mx = 0.0;
        double total_squared_error_my = 0.0;
        double total_squared_error_mz = 0.0;

        for (int i = 0; i < data_number_; ++i) {
            int row = i * 3;

            // Get measured force and torque for this pose
            double fx_i = Force_Matrix_k[row][0];
            double fy_i = Force_Matrix_k[row + 1][0];
            double fz_i = Force_Matrix_k[row + 2][0];
            double mx_measured = Torque_Matrix[row][0];
            double my_measured = Torque_Matrix[row + 1][0];
            double mz_measured = Torque_Matrix[row + 2][0];

            // Predict torque using the calculated parameters
            // τ_pred = r × F_measured + τ_bias
            double mx_predicted = (Gravity_center_y * fz_i - Gravity_center_z * fy_i) + Sensor_Erro_k1;
            double my_predicted = (Gravity_center_z * fx_i - Gravity_center_x * fz_i) + Sensor_Erro_k2;
            double mz_predicted = (Gravity_center_x * fy_i - Gravity_center_y * fx_i) + Sensor_Erro_k3;

            // Accumulate squared errors
            total_squared_error_mx += std::pow(mx_measured - mx_predicted, 2);
            total_squared_error_my += std::pow(my_measured - my_predicted, 2);
            total_squared_error_mz += std::pow(mz_measured - mz_predicted, 2);
        }

        double rmse_mx = std::sqrt(total_squared_error_mx / data_number_);
        double rmse_my = std::sqrt(total_squared_error_my / data_number_);
        double rmse_mz = std::sqrt(total_squared_error_mz / data_number_);
        double rmse_total = std::sqrt((total_squared_error_mx + total_squared_error_my + total_squared_error_mz) / (data_number_ * 3));

        RCLCPP_INFO(get_logger(), "Torque RMSE [Nm]: Mx=%.6f, My=%.6f, Mz=%.6f", rmse_mx, rmse_my, rmse_mz);
        RCLCPP_INFO(get_logger(), "Total Torque RMSE [Nm]: %.6f", rmse_total);

        // === (New) Force RMSE Calculation Step ===
        RCLCPP_INFO(get_logger(), "=== Calculating RMSE of the force model... ===");
        double total_squared_error_fx = 0.0;
        double total_squared_error_fy = 0.0;
        double total_squared_error_fz = 0.0;

        double **Tool_Weight_Vector = allocateMatrix(3, 1);
        Tool_Weight_Vector[0][0] = tw_x;
        Tool_Weight_Vector[1][0] = tw_y;
        Tool_Weight_Vector[2][0] = tw_z;

        double **Predicted_Gravity_Force = allocateMatrix(3, 1);

        for (int i = 0; i < data_number_; ++i) {
            int row = i * 3;

            // Get the transposed rotation matrix for this pose from the large matrix
            double **R_T_i = allocateMatrix(3, 3);
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    R_T_i[r][c] = Rotation_Matrix_k_calculate[row + r][c];
                }
            }

            // Predict gravity force in sensor frame: F_g = R_T * tw
            multiplyMatrix(R_T_i, 3, 3, Tool_Weight_Vector, 3, 1, Predicted_Gravity_Force);

            // Predict total force: F_pred = F_g + F_bias
            double fx_predicted = Predicted_Gravity_Force[0][0] + fx_0;
            double fy_predicted = Predicted_Gravity_Force[1][0] + fy_0;
            double fz_predicted = Predicted_Gravity_Force[2][0] + fz_0;

            // Accumulate squared errors against measured force
            total_squared_error_fx += std::pow(Force_Matrix_k[row][0] - fx_predicted, 2);
            total_squared_error_fy += std::pow(Force_Matrix_k[row + 1][0] - fy_predicted, 2);
            total_squared_error_fz += std::pow(Force_Matrix_k[row + 2][0] - fz_predicted, 2);
        }

        double rmse_fx = std::sqrt(total_squared_error_fx / data_number_);
        double rmse_fy = std::sqrt(total_squared_error_fy / data_number_);
        double rmse_fz = std::sqrt(total_squared_error_fz / data_number_);
        double rmse_force_total = std::sqrt((total_squared_error_fx + total_squared_error_fy + total_squared_error_fz) / (data_number_ * 3));

        RCLCPP_INFO(get_logger(), "Force RMSE [N]: Fx=%.6f, Fy=%.6f, Fz=%.6f", rmse_fx, rmse_fy, rmse_fz);
        RCLCPP_INFO(get_logger(), "Total Force RMSE [N]: %.6f", rmse_force_total);

        // === 5) 線上補償迴圈（與原 C++ 相同邏輯） ===
        double **Rotation_Matrix              = allocateMatrix(3, 3);
        double **Transpoed_Rotation_Matrix    = allocateMatrix(3, 3);
        double **GravityXYZ_Matrix            = allocateMatrix(3, 1);

        // 將 Tool_Weight_Matrix 更新為剛求出的 tw_x,tw_y,tw_z
        Tool_Weight_Matrix[0][0] = tw_x;
        Tool_Weight_Matrix[1][0] = tw_y;
        Tool_Weight_Matrix[2][0] = tw_z;

        rclcpp::Rate loop_rate(100.0);
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());


            tf2::Quaternion q2(
                current_pose_.pose.orientation.x,
                current_pose_.pose.orientation.y,
                current_pose_.pose.orientation.z,
                current_pose_.pose.orientation.w);
            tf2::Matrix3x3 m2(q2);
            double rx, ry, rz;
            m2.getRPY(rx, ry, rz);

            Rotation_Matrix[0][0] = std::cos(rz) * std::cos(ry);
            Rotation_Matrix[0][1] = -std::sin(rz) * std::cos(rx) + std::cos(rz) * std::sin(ry) * std::sin(rx);
            Rotation_Matrix[0][2] =  std::sin(rz) * std::sin(rx) + std::cos(rz) * std::sin(ry) * std::cos(rx);
            Rotation_Matrix[1][0] = std::sin(rz) * std::cos(ry);
            Rotation_Matrix[1][1] =  std::cos(rz) * std::cos(rx) + std::sin(rz) * std::sin(ry) * std::sin(rx);
            Rotation_Matrix[1][2] = -std::cos(rz) * std::sin(rx) + std::sin(rz) * std::sin(ry) * std::cos(rx);
            Rotation_Matrix[2][0] = -std::sin(ry);
            Rotation_Matrix[2][1] =  std::cos(ry) * std::sin(rx);
            Rotation_Matrix[2][2] =  std::cos(ry) * std::cos(rx);

            trasposeMatrix(Rotation_Matrix, Transpoed_Rotation_Matrix, 3, 3);
            multiplyMatrix(Transpoed_Rotation_Matrix, 3, 3,
                           Tool_Weight_Matrix, 3, 1,
                           GravityXYZ_Matrix);

            double fx = fx_;
            double fy = fy_;
            double fz = fz_;
            double mx = mx_;
            double my = my_;
            double mz = mz_;

            double Fex = fx - GravityXYZ_Matrix[0][0] - fx_0;
            double Fey = fy - GravityXYZ_Matrix[1][0] - fy_0;
            double Fez = fz - GravityXYZ_Matrix[2][0] - fz_0;

            double Mex = mx - GravityXYZ_Matrix[2][0] * Gravity_center_y
                             - GravityXYZ_Matrix[1][0] * Gravity_center_z - mx_0;
            double Mey = my - GravityXYZ_Matrix[0][0] * Gravity_center_z
                             - GravityXYZ_Matrix[2][0] * Gravity_center_x - my_0;
            double Mez = mz - GravityXYZ_Matrix[1][0] * Gravity_center_x
                             - GravityXYZ_Matrix[0][0] * Gravity_center_y - mz_0;

            geometry_msgs::msg::WrenchStamped out;
            out.header.stamp    = now();
            out.header.frame_id = "robotiq_ft_frame_id"; // 可依實際情況修改

            // out.wrench.force.x  = Fex;
            // out.wrench.force.y  = Fey;
            // out.wrench.force.z  = Fez;
            // out.wrench.torque.x = Mex;
            // out.wrench.torque.y = Mey;
            // out.wrench.torque.z = Mez;

            // 在發布前對補償後的結果進行濾波
            out.wrench.force.x  = filters_[0].filter(Fex);
            out.wrench.force.y  = filters_[1].filter(Fey);
            out.wrench.force.z  = filters_[2].filter(Fez);
            out.wrench.torque.x = filters_[3].filter(Mex);
            out.wrench.torque.y = filters_[4].filter(Mey);
            out.wrench.torque.z = filters_[5].filter(Mez);

            wrench_pub_->publish(out);
            loop_rate.sleep();
        }
    }

private:
    // --- callback ---
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        have_pose_ = true;
    }

    void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        // 如需與舊 ROS1 driver 軸向對齊，可在此做 swap/sign 調整
        // 輸入的 topic 已經是濾波過的，直接使用
        fx_ = msg->wrench.force.x;
        fy_ = msg->wrench.force.y;
        fz_ = msg->wrench.force.z;
        mx_ = msg->wrench.torque.x;
        my_ = msg->wrench.torque.y;
        mz_ = msg->wrench.torque.z;
        have_wrench_ = true;
    }

    void waitForFirstData()
    {
        rclcpp::Rate r(100.0);
        while (rclcpp::ok() && (!have_pose_ || !have_wrench_)) {
            rclcpp::spin_some(this->get_node_base_interface());

            r.sleep();
        }
        RCLCPP_INFO(get_logger(), "First pose & wrench received, start calibration.");
    }

    void waitForPoseStabilized()
    {
        RCLCPP_INFO(get_logger(), "Waiting for robot pose to stabilize...");

        rclcpp::Rate rate(100.0);

        geometry_msgs::msg::PoseStamped last_pose = current_pose_;
        int stable_counter = 0;

        constexpr double position_threshold = 1e-4;  // 0.1 mm
        constexpr double angle_threshold = 0.1 * M_PI / 180.0;  // 0.1 degree
        constexpr int required_stable_cycles = 100;  // 約 1 秒（100 Hz）

        while (rclcpp::ok())
        {
            rclcpp::spin_some(shared_from_this());

            double dx = current_pose_.pose.position.x - last_pose.pose.position.x;
            double dy = current_pose_.pose.position.y - last_pose.pose.position.y;
            double dz = current_pose_.pose.position.z - last_pose.pose.position.z;

            double position_delta = std::sqrt(dx*dx + dy*dy + dz*dz);
            tf2::Quaternion q_now, q_last;
            tf2::fromMsg(current_pose_.pose.orientation, q_now);
            tf2::fromMsg(last_pose.pose.orientation, q_last);

            double angle_delta = q_now.angleShortestPath(q_last);

            if (position_delta < position_threshold && angle_delta < angle_threshold) {
                stable_counter++;
            } else {
                stable_counter = 0;
            }

            if (stable_counter >= required_stable_cycles) {
                break;
            }

            last_pose = current_pose_;
            rate.sleep();
        }

        // 額外再靜止 1 秒（你一開始就要求的）
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(get_logger(), "Pose stabilized.");
    }


    void estimateInitialBias(double &fx0, double &fy0, double &fz0,
                             double &mx0, double &my0, double &mz0)
    {
        rclcpp::Rate rate_record(100.0);

        // 等第一筆非零
        while (rclcpp::ok() &&
               (std::abs(fx_) <= 0 || std::abs(fy_) <= 0 || std::abs(fz_) <= 0 ||
                std::abs(mx_) <= 0 || std::abs(my_) <= 0 || std::abs(mz_) <= 0)) {
            rclcpp::spin_some(this->get_node_base_interface());

            rate_record.sleep();
        }

        fx0 = fy0 = fz0 = mx0 = my0 = mz0 = 0.0;
        for (int i = 0; i < 200 && rclcpp::ok(); ++i) {
            rclcpp::spin_some(this->get_node_base_interface());

            fx0 += fx_;
            fy0 += fy_;
            fz0 += fz_;
            mx0 += mx_;
            my0 += my_;
            mz0 += mz_;
            rate_record.sleep();
        }
        fx0 /= 200.0;
        fy0 /= 200.0;
        fz0 /= 200.0;
        mx0 /= 200.0;
        my0 /= 200.0;
        mz0 /= 200.0;

        RCLCPP_INFO(get_logger(),
                    "Initial bias (avg 200 samples): fx0=%.6f, fy0=%.6f, fz0=%.6f, mx0=%.6f, my0=%.6f, mz0=%.6f",
                    fx0, fy0, fz0, mx0, my0, mz0);
    }

    bool sendRobotMove(const std::vector<double>& pose) {
        if (!move_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "set_positions service not available");
            return false;
        }

        auto request = std::make_shared<tm_msgs::srv::SetPositions::Request>();
        request->motion_type = tm_msgs::srv::SetPositions::Request::PTP_T;
        request->positions = pose;
        request->velocity = 0.3; // 速度不宜過快
        request->acc_time = 0.5;
        request->fine_goal = true;

        auto result = move_client_->async_send_request(request);
        
        // 等待服務回應 (Timeout 設為 20 秒)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::seconds(20)) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            return result.get()->ok;
        }
        return false;
    }

    // 成員變數
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr   wrench_pub_;
    
    rclcpp::Client<tm_msgs::srv::SetPositions>::SharedPtr move_client_;   

    geometry_msgs::msg::PoseStamped current_pose_;
    bool have_pose_   {false};
    bool have_wrench_ {false};

    double fx_{0.0}, fy_{0.0}, fz_{0.0};
    double mx_{0.0}, my_{0.0}, mz_{0.0};

    const std::vector<std::vector<double>> calibration_poses = {
        {-0.23559, -0.32411, 0.32332,  3.14159,  0.00000,  3.14159}, // 180, 0, 180
        {-0.23559, -0.32411, 0.32332, -2.37714,  0.00000,  3.14159}, // -136.2, 0, 180
        {-0.23559, -0.32411, 0.32332,  2.37191,  0.00000,  3.14159}, // 135.9, 0, 180
        {-0.23559, -0.32411, 0.32332, -2.40332,  0.51138, -2.72100}, // -137.7, 29.3, -155.9
        {-0.23559, -0.32411, 0.32332,  3.05433, -0.45900, -2.80300}, // 175.0, -26.3, -160.6
        {-0.23559, -0.32411, 0.32332,  3.01760,  0.66500, -2.50460}, // 172.9, 38.1, -143.5
        {-0.23559, -0.32411, 0.32332, -2.56740, -0.29670, -2.74020}, // -147.1, -17.0, -157.0
        {-0.23559, -0.32411, 0.32332, -2.90770,  0.72080, -2.75760}, // -166.6, 41.3, -158.0
        {-0.23559, -0.32411, 0.32332, -2.96350,  0.19020, -2.87800}  // -169.8, 10.9, -164.9
    };

    int    data_number_;
    double duration_per_pose_;
    double sample_rate_hz_;
    double tool_mass_guess_;

    std::string pose_topic_;
    std::string wrench_topic_;
    std::string output_topic_;

    std::vector<ButterworthFilter> filters_;
    // std::vector<LeadFilter> filters_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FtCompNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
