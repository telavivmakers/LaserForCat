#include <deque>
#include <vector>
#include <Arduino.h>
// for unique_ptr
#include <memory>

using namespace std;

struct Point
{
public:
  float x;
  float y;
  Point() {}
  Point(float x, float y)
  {
    this->x = x;
    this->y = y;
  }

  // add arithmetic operators + += - -= * *=
  Point operator+(const Point &p) const
  {
    return Point(x + p.x, y + p.y);
  }
  Point operator-(const Point &p) const
  {
    return Point(x - p.x, y - p.y);
  }
  Point operator*(const float &f) const
  {
    return Point(x * f, y * f);
  }
  Point &operator+=(const Point &p)
  {
    x += p.x;
    y += p.y;
    return *this;
  }
  Point &operator-=(const Point &p)
  {
    x -= p.x;
    y -= p.y;
    return *this;
  }
  Point &operator*=(const float &f)
  {
    x *= f;
    y *= f;
    return *this;
  }

};

struct QuadPoint
{
public:
  Point p0;
  Point p1;
  Point p2;
  Point p3;
  QuadPoint() {}
  QuadPoint(Point p0, Point p1, Point p2, Point p3)
  {
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = p3;
  }
};

// Catmull-Rom Spline interpolation between p1 and p2 for previous point p0 and next point p3
Point spline(const QuadPoint &quad, float t)
{
  float t2 = t * t;
  float t3 = t2 * t;

  float b1 = .5 * (-t3 + 2 * t2 - t);
  float b2 = .5 * (3 * t3 - 5 * t2 + 2);
  float b3 = .5 * (-3 * t3 + 4 * t2 + t);
  float b4 = .5 * (t3 - t2);

  return quad.p0 * b1 + quad.p1 * b2 + quad.p2 * b3 + quad.p3 * b4;
}

class SplineManouver
{
public:
  virtual QuadPoint getNextQuadPoint() = 0;
  virtual bool isFinished() = 0;
  virtual void reset() {}
  virtual ~SplineManouver() {}
};

typedef deque<Point> Points;

void addPreStartPoint(Points &points) {
  assert(points.size() > 1);
  Point p0 = points.front();
  Point p1 = points.at(1);
  Point prevP = p0 - (p1-p0);
  points.push_front(prevP);
}

void addPostEndPoint(Points &points) {
  assert(points.size() > 1);
  Point p0 = points.back();
  Point p1 = points.at(points.size() - 2);
  Point postP = p0 - (p1 - p0);
  points.push_back(postP);
}

class SplineManouverFromPoints : public SplineManouver
{
protected:
  QuadPoint getNextQuadPoint() override;
  bool isFinished() override;
  const Points &_points;
  Points::const_iterator _currentPoint;
public :
  SplineManouverFromPoints(Points &points) : _points(points) {
    _currentPoint = _points.begin();
  }
};

bool SplineManouverFromPoints::isFinished() {
  return _currentPoint + 4 == _points.end();
}

QuadPoint SplineManouverFromPoints::getNextQuadPoint() {
  QuadPoint quad = QuadPoint(*_currentPoint, *(_currentPoint + 1), *(_currentPoint + 2), *(_currentPoint + 3));
  _currentPoint++;
  return quad;
}
/*
class SlowedDownSplineManouver : public SplineManouver
{
protected:
  QuadPoint getNextQuadPoint() override;
  bool isFinished() override;
  SplineManouver &_baseSplineManouver;
  float _speed; // 0.0 < _speed <= 1.0
  QuadPoint _lastQuad;
  float _t;
  int _fractionPartOfT;
  bool _atFirstPoint = true;
  bool _atLastPoint = false;

  void advanceT() {
    float oldT = _t;
    _t += _speed;
    if (!_atFirstPoint && floor(_t) != floor(oldT)) {
      if (_baseSplineManouver.isFinished()) {
        _atLastPoint = true;
      } else {
        _lastQuad = _baseSplineManouver.getNextQuadPoint();
      }
      _atFirstPoint = false;
    }
  }

public:
  SlowedDownSplineManouver(SplineManouver &baseManouver, float speed) :
    _baseManouver(baseManouver), _speed(speed) {
      _t = -speed;
      _baseT = 0;
      _lastQuad = _baseManouver.getNextQuadPoint();
    }
};

*/


class RawManouver {
public:
  virtual bool isFinished() = 0;
  virtual Point getNextPoint() = 0;
  virtual void reset() {}
  virtual ~RawManouver() {}
};

class RepeatRawManouver : public RawManouver
{
protected:
  bool isFinished() override;
  Point getNextPoint() override;
  RawManouver &_manouver;

public:
  RepeatRawManouver(RawManouver &manouver) : _manouver(manouver) {}
  void reset() override { _manouver.reset(); }
};

bool RepeatRawManouver::isFinished()
{
  return false;
}

Point RepeatRawManouver::getNextPoint()
{
  if (_manouver.isFinished())
  {
    _manouver.reset();
  }
  return _manouver.getNextPoint();
}
class TransformRawManouver : public RawManouver
{
protected:
  bool isFinished() override;
  Point getNextPoint() override;
  Point transformPoint(Point p);
  RawManouver &_manouver;
  float _scale;
  float _rotate;
  Point _translate;
  bool _flipH;
  bool _flipV;

public:
  TransformRawManouver(RawManouver &manouver, float scale, float rotate, Point translate = Point(0, 0), bool flipH = false, bool flipV = false);
};

TransformRawManouver::TransformRawManouver(RawManouver &manouver, float scale, float rotate, Point translate, bool flipH, bool flipV) : _manouver(manouver), _scale(scale), _rotate(rotate), _translate(translate), _flipH(flipH), _flipV(flipV) {}

Point TransformRawManouver::transformPoint(Point p)
{
  if (_flipH) {
    p.x = -p.x;
  }
  if (_flipV) {
    p.y = -p.y;
  }
  float x = p.x * _scale;
  float y = p.y * _scale;
  float x1 = x * cos(_rotate) - y * sin(_rotate);
  float y1 = x * sin(_rotate) + y * cos(_rotate);
  return Point(x1 + _translate.x, y1 + _translate.y);
}

bool TransformRawManouver::isFinished()
{
  return _manouver.isFinished();
}

Point TransformRawManouver::getNextPoint()
{
  Point point = _manouver.getNextPoint();
  return transformPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////
typedef vector<RawManouver *> Manouvers;

class RawManouverSequence : public RawManouver
{
protected:
  Manouvers _manouvers;
  Manouvers::iterator _currentManouver;
public:
  RawManouverSequence(Manouvers manouvers) : _manouvers(manouvers), _currentManouver(manouvers.begin()) {}
  bool isFinished() override;
  Point getNextPoint() override;
  void reset() override;
};

bool RawManouverSequence::isFinished()
{
  return _currentManouver == _manouvers.end();
}

Point RawManouverSequence::getNextPoint()
{
  if (_currentManouver == _manouvers.end())
  {
    return Point();
  }
  Point point = (*_currentManouver)->getNextPoint();
  if ((*_currentManouver)->isFinished())
  {
    _currentManouver++;
  }
  return point;
}

void RawManouverSequence::reset()
{
  _currentManouver = _manouvers.begin();
  for (Manouvers::iterator it = _manouvers.begin(); it != _manouvers.end(); it++)
  {
    (*it)->reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////
/*
class PointwiseAddSplineManouvers : public SplineManouver
{
protected:
  Manouvers &_manouvers;

public:
  PointwiseAddSplineManouvers(Manouvers &manouvers) : _manouvers(manouvers) {}
  QuadPoint getNextQuadPoint() override;
  bool isFinished() override;
};

bool PointwiseAddSplineManouvers::isFinished()
{
  Manouvers::iterator it = _manouvers.begin();
  // if any manouver is finished, then we are finished
  while (it != _manouvers.end())
  {
    if ((*it)->isFinished())
    {
      return true;
    }
    it++;
  }
  return false;
}

QuadPoint PointwiseAddSplineManouvers::getNextQuadPoint()
{
  Manouvers::iterator it = _manouvers.begin();
  Point p0 = Point(0, 0);
  Point p1 = Point(0, 0);
  Point p2 = Point(0, 0);
  Point p3 = Point(0, 0);
  while (it != _manouvers.end())
  {
    QuadPoint quad = (*it)->getNextQuadPoint();
    p0.x += quad.p0.x;
    p0.y += quad.p0.y;
    p1.x += quad.p1.x;
    p1.y += quad.p1.y;
    p2.x += quad.p2.x;
    p2.y += quad.p2.y;
    p3.x += quad.p3.x;
    p3.y += quad.p3.y;
    it++;
  }
  return QuadPoint(p0, p1, p2, p3);
}
*/
class PointwiseAddRawManouver : public RawManouver
{
protected:
  Manouvers &_manouvers;
  Manouvers::iterator _currentManouver;

public:
  PointwiseAddRawManouver(Manouvers &manouvers) : _manouvers(manouvers), _currentManouver(manouvers.begin()) {}
  bool isFinished() override;
  Point getNextPoint() override;
  void reset() override;
};

bool PointwiseAddRawManouver::isFinished()
{
  return _currentManouver == _manouvers.end();
}

Point PointwiseAddRawManouver::getNextPoint()
{
  if (_currentManouver == _manouvers.end())
  {
    return Point();
  }
  Point point = (*_currentManouver)->getNextPoint();
  if ((*_currentManouver)->isFinished())
  {
    _currentManouver++;
  }
  while (_currentManouver != _manouvers.end())
  {
    Point nextPoint = (*_currentManouver)->getNextPoint();
    point.x += nextPoint.x;
    point.y += nextPoint.y;
    if (!(*_currentManouver)->isFinished())
    {
      break;
    }
    _currentManouver++;
  }
  return point;
}

void PointwiseAddRawManouver::reset()
{
  _currentManouver = _manouvers.begin();
  for (Manouvers::iterator it = _manouvers.begin(); it != _manouvers.end(); it++)
  {
    (*it)->reset();
  }
}
//////////////////////////////////////////////////////////////////////////////////

constexpr float epsilon = 0.0001;

class RawManouverFromSplineManouver : public RawManouver {
protected:
  SplineManouver& _splineManouver;
  bool _SplineInputFinished = false;
  float _t = 0;
  QuadPoint _lastQuad;
  float _fractionPartOfT;
  float _rate;

  void advanceT() {
    float oldT = _t;
    _t += _rate;
    if (floor(_t+epsilon) != floor(oldT+epsilon)) {
      if (_splineManouver.isFinished())
      {
        _SplineInputFinished = true;
      }
      else
      {
        _lastQuad = _splineManouver.getNextQuadPoint();
      }
    }
    _fractionPartOfT = _t - floor(_t+epsilon);
  }

  void partialReset() {
    _SplineInputFinished = false;
    _t = 0;
    _lastQuad = _splineManouver.getNextQuadPoint();
    _fractionPartOfT = 0;
  }

public:
  void reset() override {
    partialReset();
    _splineManouver.reset();
  }
  RawManouverFromSplineManouver(SplineManouver& splineManouver, float rate) :
      _splineManouver(splineManouver), _rate(rate) {
      partialReset();
  }
  bool isFinished() override {
    if (!_SplineInputFinished) {
      return false;
    }
    return _fractionPartOfT + epsilon > 1.0 + _rate / 2.0;
  }
  Point getNextPoint() override {
    advanceT();
    return spline(_lastQuad, _fractionPartOfT);
  }
};

class linearRawManouver : public RawManouver {
protected:
  Point _p1;
  float _t = 0;
  float _rate;
  Point totalDelta;

  void advanceT() {
    float oldT = _t;
    _t += _rate;
  }
public:
  linearRawManouver(Point p1, Point p2, int numberOfSteps) : _p1(p1),  {
    _rate = 1.0 / numberOfSteps;
    totalDelta = p2 - p1;
  }

  bool isFinished() override {
    return _t + epsilon > 1.0;
  }

  Point getNextPoint() override {
    advanceT();
    return _p1 + totalDelta * _t;
  }
};


int main()
{
  Points points = Points();
  points.push_back(Point(0, 0));
  points.push_back(Point(1, 1));
  points.push_back(Point(2, 0));
  points.push_back(Point(3, 1));
  points.push_back(Point(4, 0));
  points.push_back(Point(5, 1));
  points.push_back(Point(6, 0));
  points.push_back(Point(7, 1));
  points.push_back(Point(8, 0));
  points.push_back(Point(9, 1));
  points.push_back(Point(10, 0));
  points.push_back(Point(11, 1));
  points.push_back(Point(12, 0));
  points.push_back(Point(13, 1));
  points.push_back(Point(14, 0));
  points.push_back(Point(15, 1));
  points.push_back(Point(16, 0));
  points.push_back(Point(17, 1));
  points.push_back(Point(18, 0));
  points.push_back(Point(19, 1));
  points.push_back(Point(20, 0));
  points.push_back(Point(21, 1));
  points.push_back(Point(22, 0));
  points.push_back(Point(23, 1));
  points.push_back(Point(24, 0));
  points.push_back(Point(25, 1));
  points.push_back(Point(26, 0));
  points.push_back(Point(27, 1));
  points.push_back(Point(28, 0));
  points.push_back(Point(29, 1));
  points.push_back(Point(30, 0));
  points.push_back(Point(31, 1));
  points.push_back(Point(32, 0));
  points.push_back(Point(33, 1));
  points.push_back(Point(34, 0));
  points.push_back(Point(35, 1));
  points.push_back(Point(36, 0));
  points.push_back(Point(37, 1));
  points.push_back(Point(38, 0));
  points.push_back(Point(39, 1));
  points.push_back(Point(40, 0));
  points.push_back(Point(41, 1));
  points.push_back(Point(42, 0));
  points.push_back(Point(43, 1));
  points.push_back(Point(44, 0));
}
