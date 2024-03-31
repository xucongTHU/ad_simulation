
//
// Copyright (c) 2024 xucong Authors. All rights reserved.
// Created by xucong on 24-3-12.
//
#ifndef PARKINGSPACE_FREESPACE_COMMON_GEOMETRY_UTIL_H_
#define PARKINGSPACE_FREESPACE_COMMON_GEOMETRY_UTIL_H_
#include <string>
#include "third_party/apollo/proto/geometry/geometry.pb.h"
#include "third_party/apollo/proto/hdmap/map_geometry.pb.h"
#include <valarray>

namespace stoic::app::core {
// rotate surround origin(0,0)
inline bool rotate(double srcX, double srcY, double rotRadAng, double& outX, double& outY)
{
  outX = srcX * cos(rotRadAng) - srcY * sin(rotRadAng);
  outY = srcX * sin(rotRadAng) + srcY * cos(rotRadAng);
}
//just check x/y , angRange:in Deg
inline bool isSameDirect(const apollo::geometry::PointENU& vec1, const apollo::geometry::PointENU& vec2, double angRange)
{
  double length1 = sqrt(vec1.x()*vec1.x() + vec1.y()*vec1.y());
  double nor_vec1_x = vec1.x() / length1;
  double nor_vec1_y = vec1.y() / length1;
  double length2 = sqrt(vec2.x()*vec2.x() + vec2.y()*vec2.y());
  double nor_vec2_x = vec2.x() / length2;
  double nor_vec2_y = vec2.y() / length2;
  double cosa = nor_vec1_x * nor_vec2_x + nor_vec1_y * nor_vec2_y;
  if(cosa > 1.0)
  {
    cosa = 1.0;
  }else if(cosa < -1.0)
  {
    cosa = -1.0;
  }
  double angInDeg = acos(cosa) * 180.0/M_PI;
  return (angInDeg < angRange);
}
inline double distance(const apollo::geometry::PointENU& pntF, const apollo::geometry::PointENU& pntT)
{
  return sqrt((pntF.x() - pntT.x()) * (pntF.x() - pntT.x()) + (pntF.y() - pntT.y())*  (pntF.y() - pntT.y()));
}
inline bool inSquare(const apollo::geometry::PointENU& pntTest, const apollo::geometry::PointENU& pntRef, double range)
{
  return abs(pntTest.x() - pntRef.x()) < range && abs(pntTest.y() - pntRef.y()) < range;
}
inline bool isProjectToLine(const apollo::geometry::PointENU& test, const apollo::geometry::PointENU& pntS, const apollo::geometry::PointENU& pntE)
{
  if(abs(pntS.x() - pntE.x()) < 1.0E04 && abs(pntS.y() - pntE.y())<1.0E04)
  {
    return true;
  }
  double a2 = (test.x() - pntS.x()) * (test.x() - pntS.x()) + (test.y() - pntS.y()) *  (test.y() - pntS.y());
  double b2 = (test.x() - pntE.x()) * (test.x() - pntE.x()) + (test.y() - pntE.y()) *  (test.y() - pntE.y());
  double c2 = (pntE.x() - pntS.x()) * (pntE.x() - pntS.x()) + (pntE.y() - pntS.y()) *  (pntE.y() - pntS.y());
  if(b2+c2<a2 || a2+c2<b2)
  {
    return false;
  }
  else
  {
    return true;
  }
}
inline apollo::geometry::PointENU getPtToLine(const apollo::geometry::PointENU& test, const apollo::geometry::PointENU& pntS, const apollo::geometry::PointENU& pntE)
{
  if(abs(pntS.x() - pntE.x()) < 1.0E04 && abs(pntS.y() - pntE.y())<1.0E04)
  {
    return pntS;
  }
  apollo::geometry::PointENU pntMid;
  double dDeltaX = pntE.x() - pntS.x();
  double dDeltaY = pntS.y() - pntE.y();
  double dDeltaX2 = dDeltaX*dDeltaX;
  double dDeltaY2 = dDeltaY * dDeltaY;
  double dDeltaXY = dDeltaX * dDeltaY;
  double dLineSectDist = dDeltaX2 + dDeltaY2;
  pntMid.set_x( (dDeltaXY* (pntS.y() - test.y()) + pntS.x() * dDeltaY2 + test.x() * dDeltaX2)/dLineSectDist );
  pntMid.set_y( (dDeltaXY*(pntS.x() - test.x())  + pntS.y()*dDeltaX2 + test.y()*dDeltaY2)/dLineSectDist );
  pntMid.set_z((pntS.z() + pntE.z())*0.5);
  return pntMid;
}

inline double getNearestDistance(const apollo::geometry::PointENU& ego_position, const ::apollo::hdmap::LineSegment& line, double distance_range,
                                 apollo::geometry::PointENU& headVector, apollo::geometry::PointENU& nearestPt)
{
  double dMinDis = -1.0;
  double dDisTemp;
  apollo::geometry::PointENU pntProject;
  const ::google::protobuf::RepeatedPtrField< ::apollo::geometry::PointENU >& refPoints = line.point();
  for(int i=0;i<refPoints.size()-1;i++)
  {
    const apollo::geometry::PointENU& pntFrom = refPoints.Get(i);
    const apollo::geometry::PointENU& pntTo = refPoints.Get(i+1);
    if(!inSquare(pntFrom, ego_position, distance_range) )
    {
      continue;
    }
    if(isProjectToLine(ego_position, pntFrom, pntTo))
    {
      pntProject = getPtToLine(ego_position, pntFrom, pntTo);
      dDisTemp = distance(ego_position, pntProject);
    }else{
      continue;
    }
    if(dMinDis < 0.0 || dDisTemp < dMinDis)
    {
      dMinDis = dDisTemp;
      headVector.set_x(pntTo.x() - pntFrom.x());
      headVector.set_y(pntTo.y() - pntFrom.y());
      headVector.set_z(pntTo.z() - pntFrom.z());
      nearestPt = pntProject;
    }
  }
  for(int i=0;i<refPoints.size();i++)
  {
    if(!inSquare(refPoints.Get(i), ego_position, distance_range) )
    {
      continue;
    }
    dDisTemp = distance(ego_position, refPoints.Get(i));
    if(dMinDis < 0.0 || dDisTemp < dMinDis)
    {
      dMinDis = dDisTemp;
      apollo::geometry::PointENU pntFrom;
      apollo::geometry::PointENU pntTo;
      if(i<refPoints.size()-1)
      {
        pntFrom = refPoints.Get(i);
        pntTo = refPoints.Get(i+1);
      }
      else{
        pntFrom = refPoints.Get(i-1);
        pntTo = refPoints.Get(i);
      }
      headVector.set_x(pntTo.x() - pntFrom.x());
      headVector.set_y(pntTo.y() - pntFrom.y());
      headVector.set_z(pntTo.z() - pntFrom.z());
      nearestPt = pntFrom;
    }
  }
  return dMinDis;
}
inline bool isPtOnLine(const apollo::geometry::PointENU& pntTest, const apollo::geometry::PointENU& pntFrom,
                       const apollo::geometry::PointENU& pntTo, const double& disRange)
{
  if(pntFrom.x() == pntTo.x() && pntFrom.y() == pntTo.y())
  {
    return false;
  }
  if(abs(pntTo.x() - pntFrom.x())< disRange)
  {
    return abs(pntTo.x() - pntTest.x())<disRange;
  }
  if(abs(pntTo.y() - pntFrom.y())< disRange)
  {
    return abs(pntTo.y() - pntTest.y())<disRange;
  }
  if((pntFrom.x() == pntTest.x() && pntFrom.y() == pntTest.y()) ||
      (pntTo.x() == pntTest.x() && pntTo.y() == pntTest.y()) )
  {
    return true;
  }
  if(abs(pntTo.x() - pntTest.x())< disRange)
  {
    return false;
  }
  double dRes = (pntTest.y() - pntTo.y()) * (pntTo.x() - pntFrom.x()) - (pntTo.y() - pntFrom.y())* (pntTest.x() - pntTo.x());
  dRes /= ((pntTo.x() - pntFrom.x()) * (pntTest.x() - pntTo.x()));
  if(abs(dRes) < disRange)
  {
    return true;
  }
  else{
    return false;
  }
}
//计算点在线的左右侧，０　线上，１　左侧，２　右侧
inline int pntMatchLine(const apollo::geometry::PointENU& pntTest, const apollo::geometry::PointENU& pntFrom, const apollo::geometry::PointENU& pntTo)
{
  if(isPtOnLine(pntTest, pntFrom, pntTo, 0.1))
  {
    return 0;
  }
  double dRes = (pntTest.y() - pntTo.y()) * (pntTo.x() - pntFrom.x()) - (pntTo.y() - pntFrom.y())* (pntTest.x() - pntTo.x());
  if(dRes < 0)
  {
    return 2;
  }else
  {
    return 1;
  }
}

}
#endif //PARKINGSPACE_FREESPACE_COMMON_GEOMETRY_UTIL_H_
