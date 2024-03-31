
#include "app/simulator/vtd_integration/front_vision/lane_line_type/lane_boundary.h"

namespace simulator {
namespace vtd_integration {

double LaneBoundary::RawLineStart() const {
  return raw_line_.poly.start_x;
}

double LaneBoundary::RawLineEnd() const {
  return raw_line_.poly.end_x;
}

double LaneBoundary::LineStart() const {
  return start_x;
}

double LaneBoundary::LineEnd() const {
  return end_x;
}

double LaneBoundary::GetLineRange() const {
  return fabs(end_x - start_x);
}

float LaneBoundary::GetLineMarkWidth() const {
  // if (line_mark_width > 1.0) SetLineMarkWidth(0.0f);
  return line_mark_width;
}

// void LaneBoundary::TransposeXY(){
//   std::vector<caic_std::Point2d> points__;
//   start_x = 0;
//   end_x = 20;
//   double temp_x = 0;
//   for (auto point: points_){
//     if (point.y < 0 || point.y > 50){continue;}
//     temp_x = point.y;
//     start_x = temp_x > start_x ? start_x: point.y;
//     end_x = temp_x > end_x ? point.y: end_x;

//     caic_std::Point2d pt = {point.y, -point.x};
//     points__.emplace_back(pt);
//   }
//   points_.clear();
//   points_ = points__;
// }

void LaneBoundary::TransposeXY(){
  /*远处降频采样，并设置Valid状态位*/
  int size_p = points_.size();

  if (size_p < 4){  // 节点个数第一次筛选
    SetValid(false);
    return;
  }

  std::vector<caic_std::Point2d> points__;

  start_x = points_[0].y;
  end_x = points_[size_p-1].y;
  if (start_x > end_x){
    double temp = end_x;
    end_x = start_x;
    start_x = temp;
  }

  start_x = start_x < 0 ? 0 : start_x;

  double temp_x = 0;

  int miu = 0;   // 降频采样索引参数
  int t_miu = 0; // 降频采样开始索引

  int rear_pts = 3;
  int front_pts = 10;  // 前方连续采点个数

  caic_std::Point2d start_p = {points_[0].y, -points_[0].x};  // 初始化起点
  for (int i = 0; i < size_p; i++){
    // 条件截断操作
    if (points__.empty()){
      start_p = {points_[i].y, -points_[i].x};
    }
    else{
      caic_std::Point2d end_p = {points_[i].y, -points_[i].x};
      double slope = CalSlope(start_p, end_p);  // tan(80)=5.67 tan(60)=1.73
      double delta = points_[i].y - start_p.x;  // 滤除u形弯对向车道线
      // LOG_INFO("slope: %lf", slope);
      if (std::abs(slope) > 1.73 || delta < 0){
        break;  // 截断操作
      }
      start_p = {points_[i].y, -points_[i].x};
    }
    // 车体后方采点3m
    if (points_[i].y < 0){
      if(points_[i].y > -3){
        caic_std::Point2d pt = {points_[i].y, -points_[i].x};
        points__.emplace_back(pt);
      }
      continue;
    }
    // 前方连续采点范围
    if (front_pts > 0 || (points_[i].y - start_x) < 20 ){
      front_pts --;
      // LOG_INFO("(points_[i].y - start_x): %lf, front_pts: %d", (points_[i].y - start_x), front_pts);
      // start_x = points_[i].y > start_x ? start_x : points_[i].y;
      // end_x = points_[i].y > end_x ? points_[i].y : end_x;

      caic_std::Point2d pt = {points_[i].y, -points_[i].x};
      points__.emplace_back(pt);
      t_miu = i;
      continue;
    }
    // 前方降频采点范围
    if (i == t_miu + std::pow(2, miu) || i == size_p - 1){  // 远处点按等差数列间距取点
    // // }else if (i == t_miu + 8 * miu){
    //   LOG_INFO("miu: %d", miu);

      // start_x = points_[i].y > start_x ? start_x : points_[i].y;
      // end_x = points_[i].y > end_x ? points_[i].y : end_x;

    //   LOG_INFO("points_[i].y > 20, i: %d, start_x: %lf, end_x: %lf", i, start_x, end_x);

      caic_std::Point2d pt = {points_[i].y, -points_[i].x};
      points__.emplace_back(pt);
      miu++;
      continue;
    }
    // else{
    //   LOG_INFO("points_[i].y > 20, i: %d", i);
    //   continue;
    //  }
  }

  if (points__.size() < 4){  // 节点个数第二次筛选
    SetValid(false);
    start_x = 0;
    end_x = 0;
    return;
  }
  else{
    SetValid(true);
  }

  // LOG_INFO("TransposeXY start_x: %lf, end_x: %lf, Valid: %d", start_x, end_x, Valid());

  points_.clear();
  points_ = points__;
}

void LaneBoundary::GetDomain(){

}

caic_perception::LaneType LaneBoundary::GetLaneType() const {
  return lane_line_type;
}

const cubic_polynomial& LaneBoundary::Polynomial() const {
  return polynomial_;
}

void LaneBoundary::ComputeWidth(bool as_left, const double& x) {

}

void LaneBoundary::Sample(const double& start, const double& end, const double& delta) {
}

void LaneBoundary::Resample(){
  points_.clear();

  if (!Valid()){
    // std::cout<<"resample return"<<std::endl;
    return;
  }

  float delta = 1;
  // RawLine& raw_line__ = GetRawLine();
  float start_x = raw_line_.poly.start_x;
  float end_x = raw_line_.poly.end_x;
  float c0 = raw_line_.poly.c0;
  float c1 = raw_line_.poly.c1;
  float c2 = raw_line_.poly.c2;
  float c3 = raw_line_.poly.c3;


  // printf("length: %f \n", end_x - start_x);

  for(float x = start_x; x < end_x; x+=delta){
    float y = c0 + c1 * x + c2 * x * x + c3 * x * x * x;
    caic_std::Point2d point = {(double)x, (double)y};
    points_.emplace_back(point);
    // std::cout<<"resample x: "<<x<<"\n"
    //           <<"resample y: "<<y<<"\n"<<std::endl;
  }
}

void LaneBoundary::FitPoly(){
  int orders = 3;

  if (points_.size() < 4){
    SetValid(false);
    return;
  }

  std::vector<caic_std::Point2d> fit_points = points_;

  // 加权实现
  int size_pts = points_.size();
  int int_w = size_pts;
  for(int i = 0;i < size_pts; i++){
    caic_std::Point2d point_ = points_[i];
    int w_ = int_w;
    while (w_>0)
    {
      w_ --;
      fit_points.emplace_back(point_);
    }
    int_w --;
  }

  // int size_addition = addition_points_.size();
  // for (int i = 0;i < size_addition;i++){
  //   for(int j = 0;j < 2;j++){
  //     fit_points.emplace_back(addition_points_[i]);
  //   }
  //   for(int j = 0;j < 4;j++){
  //     fit_points.emplace_back(points_[j]);
  //   }
  // }

  // cv::Mat ret;
  // polynomial_curve_fit(fit_points, 3, ret);

  cv::Mat ret = polyfit_pow(fit_points, 3);

  // LOG_INFO("ret.at<double>(0), ret.at<double>(1), ret.at<double>(2), ret.at<double>(3): %lf, %lf, %lf, %lf",
  //           ret.at<double>(0), ret.at<double>(1), ret.at<double>(2), ret.at<double>(3));

  // Eigen::VectorXd ret = FitUseRANSAC(orders, x_in_ego_coordi, y_in_ego_coordi);

  polynomial_ = cubic_polynomial(ret.at<double>(0), ret.at<double>(1), ret.at<double>(2), ret.at<double>(3));
}

cv::Mat LaneBoundary::polyfit_pow(std::vector<caic_std::Point2d>& in_point, int n) {
  int size = in_point.size();
  //所求未知数个数
  int x_num = n + 1;
  //构造矩阵U和Y
  cv::Mat mat_u(size, x_num, CV_64F);
  cv::Mat mat_y(size, 1, CV_64F);

  for (int i = 0; i < mat_u.rows; ++i)
    for (int j = 0; j < mat_u.cols; ++j)
    {
      mat_u.at<double>(i, j) = pow(in_point[i].x, j);
    }

  for (int i = 0; i < mat_y.rows; ++i)
  {
    mat_y.at<double>(i, 0) = in_point[i].y;
  }

  //矩阵运算，获得系数矩阵K
  cv::Mat mat_k(x_num, 1, CV_64F);
  mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
  return mat_k;
}

// void LaneBoundary::polyfit_ceres(std::vector<double> &x_data, std::vector<double> &y_data, double[4] para){

//     int N = x_data.size();

//     // 构建最小二乘问题
//     ceres::Problem problem;
//     for (int i=0; i<N; i++) {
//         // 向问题中添加误差项
//         problem.AddResidualBlock(
//             // 使用自动求导
//             // 模板参数：误差类型、输出维度、输入维度，维数要与前面struct中一致
//             // 因为要优化的是a, b, c三个量，且都是标量，所以是1x3，配置维度为1，3
//             new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
//                 new CURVE_FITTING_COST(x_data[i], y_data[i])
//             ),
//             // 核函数，不使用则配置为空指针
//             nullptr,
//             // 待估计参数
//             para
//         );
//     }

//     // 配置求解器
//     // 这里有很多配置项可以填，包括迭代次数，步长，使用的方法等等……
//     // 可以参考Ceres的Options详解：https://blog.csdn.net/DumpDoctorWang/article/details/84890792
//     std::ceres::Solver::Options options;
//     // 配置最大迭代次数（默认50），但这只是最大值，程序可能提前停止
//     options.max_num_iterations = 50;
//     // 求解优化问题有两类方法：信任域（TRUST_REGION）和线性搜索（LINE_SEARCH，尚不支持边界约束）
//     // options.minimizer_type默认TRUST_REGION

//     // 增量方程如何求解
//     options.linear_solver_type = std::ceres::DENSE_NORMAL_CHOLESKY;
//     // 设置输出到cout
//     options.minimizer_progress_to_stdout = true;

//     // 优化信息
//     std::ceres::Solver::Summary summary;

//     // 开始优化
//     std::ceres::Solve(options, &problem, &summary);

//     // 在屏幕上输出结果
//     std::cout << summary.BriefReport() << std::endl;
//     std::cout << "a, b, c 为：";
//     for (auto a:para) std::cout << a << " ";
//     std::cout << std::endl;
//  }

void LaneBoundary::polynomial_curve_fit(std::vector<caic_std::Point2d>& key_point, int n, cv::Mat& A)
    {
        //Number of key points
        int N = key_point.size();

        //构造矩阵X
        cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
        for (int i = 0; i < n + 1; i++)
        {
            for (int j = 0; j < n + 1; j++)
            {
                for (int k = 0; k < N; k++)
                {
                    X.at<double>(i, j) = X.at<double>(i, j) +
                        std::pow(key_point[k].x, i + j);
                }
            }
        }

        //构造矩阵Y
        cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
        for (int i = 0; i < n + 1; i++)
        {
            for (int k = 0; k < N; k++)
            {
                Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                    std::pow(key_point[k].x, i) * key_point[k].y;
            }
        }

        A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
        //求解矩阵A
        cv::solve(X, Y, A, cv::DECOMP_SVD);
    }

// 矩阵方式求解
void LaneBoundary::PolyFit(const std::vector<caic_std::Point2d> &points, cv::Mat& coeff)
{
  const int order = 3;
  const int n = points.size();
  cv::Mat A = cv::Mat::ones(n, order + 1, CV_64FC1);
  cv::Mat B = cv::Mat::zeros(n, 1, CV_64FC1);

  for (int i = 0; i < n; ++i)
  {
    const double a = points.at(i).x;
    const double b = points.at(i).y;
    B.at<double>(i, 0) = b;
    if (i > 0)
    {
      for (int j = 1, v = a; j < order + 1; ++j, v *= a)
      {
        A.at<double>(i, j) = v;
      }
    }
  }

  coeff = (A.t() * A).inv() * A.t() * B;
}

// later ransac should be introduced to find the inliner and outliner points
Eigen::VectorXd LaneBoundary::FitUseRANSAC(const int& orders, std::vector<double> x_in_ego_coordi, std::vector<double> y_in_ego_coordi){

    Eigen::VectorXd ret;
    ret.resize(orders + 1);

    Eigen::Map<Eigen::VectorXd> sample_x(x_in_ego_coordi.data(), x_in_ego_coordi.size());
    Eigen::Map<Eigen::VectorXd> sample_y(y_in_ego_coordi.data(), y_in_ego_coordi.size());

    Eigen::MatrixXd mtx_vander_monde(
        x_in_ego_coordi.size(),
        orders + 1);  // Vandermonde matrix of X-axis coordinate vector of sample data
    Eigen::VectorXd col_vander_monde = sample_x;  // Vandermonde column

    // construct Vandermonde matrix column by column
    for (size_t i = 0; i < orders + 1; ++i) {
      if (0 == i) {
        mtx_vander_monde.col(0) = Eigen::VectorXd::Constant(x_in_ego_coordi.size(), 1, 1);
        continue;
      }
      if (1 == i) {
        mtx_vander_monde.col(1) = col_vander_monde;
        continue;
      }

      col_vander_monde = col_vander_monde.array() * sample_x.array();
      mtx_vander_monde.col(i) = col_vander_monde;
    }

    ret = mtx_vander_monde.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(sample_y);

    // check loss to reduce the poly fit distance later
    return ret;
}

std::string LaneBoundary::Id() const {
  if (raw_line_.id == 0) {
    return "";
  } else {
    return std::to_string(raw_line_.id);
  }
}

void LaneBoundary::UpdateCompensatedPoly(const PlanningPose& relative_pose) {
    polynomial_compensated_.MutableC0() = polynomial_.Evaluate(relative_pose.translation[0], 0) - relative_pose.translation[1];
    polynomial_compensated_.MutableC1() = atan(polynomial_.Evaluate(relative_pose.translation[0], 1)) - asin(relative_pose.rotation(0, 1));
  }


}
}

