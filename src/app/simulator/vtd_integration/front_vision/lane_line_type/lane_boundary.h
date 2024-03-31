#pragma once


#include <math.h>
#include <memory>

#include "simulator/vtd_integration/front_vision/lane_line_type/cubic_polynomial.h"
#include "simulator/vtd_integration/front_vision/lane_line_type/point.h"
#include "simulator/vtd_integration/front_vision/lane_line_type/lane_fuision_data_type.h"
#include "common/log/Logger.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace simulator {
namespace vtd_integration {

using RawLine = caic_perception::PerceptionLane;


class LaneBoundary {
 public:
  LaneBoundary(RawLine& raw_line, const LineSourceType line_source_type, const int64_t time_stamp) :
      polynomial_(raw_line.poly.c0,
                  raw_line.poly.c1,
                  raw_line.poly.c2,
                  raw_line.poly.c3),
                  line_mark_width(raw_line.width),
                  lane_line_type(raw_line.type),
                  polynomial_compensated_(polynomial_),
                  polynomial_filted_(polynomial_),
                  raw_line_(raw_line),
                  line_source_type_(line_source_type),
                  time_stamp_(time_stamp){

                    if (raw_line.score > 0.9){
                      SetValid(true);
                    }
                    else{
                      SetValid(false);
                    }

                    if (line_source_type_ == LineSourceType::HDMAP && raw_line.score > 0.7){
                  
                      for(int i = 0;i < raw_line_.points_size;i++){
                        points_.emplace_back(raw_line_.points[i]);
                      }
                      TransposeXY();
                      SetLineMarkWidth(0.0f);
                    }
                    else if (line_source_type_ == LineSourceType::HDMAP){
                      SetValid(false);
                    }

                    if(line_source_type_ == LineSourceType::CAMERA){
                      end_x = raw_line.poly.end_x;
                      start_x = raw_line.poly.start_x;
                    }
                  }

 public:
  std::string Id() const;

  double RawLineStart() const;
  double RawLineEnd() const;

  double LineStart() const;
  double LineEnd() const;

  caic_std::Point2d GetPoint(int i){return points_[i];}

  caic_std::Point2d GetEndPoint(int i){return points_.at(points_.size()-1-i);}

  void PushAdditionPoint(caic_std::Point2d &point){addition_points_.emplace_back(point);}

  void SetEndX(double x){end_x = x;}

  double GetLineRange() const;

  double GetStartX() const {return start_x;}

  double GetEndX() const {return end_x;}

  float GetStartXf() const {return float(start_x);}

  float GetEndXf() const {return float(end_x);}

  std::vector<caic_std::Point2d> GetPoints() const {return points_;}

  void SetLineMarkWidth(float width) {line_mark_width = width;}

  float GetLineMarkWidth() const;

  void TransposeXY();

  void GetDomain();

  caic_perception::LaneType GetLaneType() const;

  double Score() const {return raw_line_.score;}

  uint32_t LineID() const {return raw_line_.id;}

  bool Valid() const{
    // const double kValidThres = 0.7;
    return valid;
  }

  void SetValid(bool valid_){
    valid = valid_;
  }

  const cubic_polynomial& Polynomial() const;

  void UpdateCompensatedPoly(const PlanningPose& relative_pose);
  const cubic_polynomial& CompensatedPoly()const {return polynomial_compensated_;}

  cubic_polynomial& MutableFiltedPoly() {return polynomial_filted_;}
  const cubic_polynomial& FiltedPoly()const {return polynomial_filted_;}

  void ComputeWidth(bool as_left, const double& x);
  void Sample(const double& start, const double& end, const double& delta);

  const RawLine& GetRawLine() const { return raw_line_; }

  const double CalSlope(const caic_std::Point2d& start_p, caic_std::Point2d& end_p) const{
    return (end_p.y - start_p.y)/(end_p.x - start_p.x);
  }

  // RawLine& GetRawLine() { return raw_line_; }

  const int64_t& TimeStamp() const{ return time_stamp_;}

  void Resample();

  void FitPoly();

  Eigen::VectorXd FitUseRANSAC(const int&, std::vector<double>, std::vector<double>);
  void PolyFit(const std::vector<caic_std::Point2d> &, cv::Mat &);

  cv::Mat polyfit_pow(std::vector<caic_std::Point2d>& , int);

  void polynomial_curve_fit(std::vector<caic_std::Point2d>& key_point, int n, cv::Mat& A);


 private:
//  // 代价函数的计算模型
//   struct CURVE_FITTING_COST {
//       CURVE_FITTING_COST(double x, double y): _x(x), _y(y) {}

//       // 残差的计算
//       // 表示T在模版实例化时可以替换任意类型，不仅包括内置类型，也包括自定义类型
//       template<typename T>
//       // abc是3维的模型参数
//       // 用operator关键词重载了()运算符，目的是将类打造为一个可以和函数一样调用的拟函数
//       // 对实例化的类对象a可以直接调用a<double>()方法
//       // 接收的参数将是Ceres传递过来的雅可比矩阵，用于自动求导
//       bool operator() (const T *const abc, T *residual) const {
//               // y-exp(ax^2+bx+c)
//               // 使用ceres::exp计算e
//               residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
//               return true;
//           }

//       // x, y数据
//       const double _x, _y;
//   };

//   void LaneBoundary::polyfit_ceres(std::vector<double> &, std::vector<double> &, double[4]);


 private:
  cubic_polynomial polynomial_;
  cubic_polynomial polynomial_compensated_;
  cubic_polynomial polynomial_filted_;
  RawLine raw_line_;
  double end_x = 0;
  double start_x = 0;
  float line_mark_width;
  bool valid = false;
  caic_perception::LaneType lane_line_type;
  // RawLine& raw_line_;
  LineSourceType line_source_type_ = LineSourceType::None;
  int64_t time_stamp_ = 0;
  std::vector<caic_std::Point2d> points_;
  std::vector<caic_std::Point2d> addition_points_;  // 保存添加进拟合的外来点集
};

using LaneBoundaryPtr = std::shared_ptr<LaneBoundary>;

} // lanefusion
}


