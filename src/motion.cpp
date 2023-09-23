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

  float x = quad.p0.x * b1 + quad.p1.x * b2 + quad.p2.x * b3 + quad.p3.x * b4;
  float y = quad.p0.y * b1 + quad.p1.y * b2 + quad.p2.y * b3 + quad.p3.y * b4;

  return Point(x, y);
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
  Point prevP = Point(p0.x - (p1.x - p0.x), p0.y - (p1.y - p0.y));
  points.push_front(prevP);
}

void addPostEndPoint(Points &points) {
  assert(points.size() > 1);
  Point p0 = points.back();
  Point p1 = points.at(points.size() - 2);
  Point postP = Point(p0.x + (p0.x - p1.x), p0.y + (p0.y - p1.y));
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

class RepeatSplineManouver : public SplineManouver
{
protected:
  QuadPoint getNextQuadPoint() override;
  bool isFinished() override;
  SplineManouver &_manouver;

public:
  RepeatSplineManouver(SplineManouver &manouver) : _manouver(manouver) {}
  void reset() override { _manouver.reset(); }
};

bool RepeatSplineManouver::isFinished()
{
  return false;
}

QuadPoint RepeatSplineManouver::getNextQuadPoint()
{
  if (_manouver.isFinished())
  {
    _manouver.reset();
  }
  return _manouver.getNextQuadPoint();
}

class TransformSplineManouver : public SplineManouver
{
protected:
  QuadPoint getNextQuadPoint() override;
  bool isFinished() override;
  Point transformPoint(Point p);
  SplineManouver &_manouver;
  float _scale;
  float _rotate;
  Point _translate;

public:
  TransformSplineManouver(SplineManouver &manouver, float scale, float rotate, Point translate = Point(0, 0));
};

TransformSplineManouver::TransformSplineManouver(SplineManouver &manouver, float scale, float rotate, Point translate) : _manouver(manouver), _scale(scale), _rotate(rotate), _translate(translate) {}

Point TransformSplineManouver::transformPoint(Point p)
{
  float x = p.x * _scale;
  float y = p.y * _scale;
  float x1 = x * cos(_rotate) - y * sin(_rotate);
  float y1 = x * sin(_rotate) + y * cos(_rotate);
  return Point(x1 + _translate.x, y1 + _translate.y);
}

QuadPoint TransformSplineManouver::getNextQuadPoint()
{
  QuadPoint quad = _manouver.getNextQuadPoint();
  return QuadPoint(
      transformPoint(quad.p0),
      transformPoint(quad.p1),
      transformPoint(quad.p2),
      transformPoint(quad.p3));
}

bool TransformSplineManouver::isFinished()
{
  return _manouver.isFinished();
}

typedef vector<SplineManouver *> Manouvers;
class SplineManouverSequence : public SplineManouver
{
protected:
  QuadPoint getNextQuadPoint() override;
  bool isFinished() override;
  Manouvers _manouvers;
  Manouvers::iterator _currentManouver;
};

bool SplineManouverSequence::isFinished()
{
  return _currentManouver == _manouvers.end();
}

QuadPoint SplineManouverSequence::getNextQuadPoint()
{
  if (_currentManouver == _manouvers.end())
  {
    return QuadPoint();
  }
  QuadPoint quad = (*_currentManouver)->getNextQuadPoint();
  if ((*_currentManouver)->isFinished())
  {
    _currentManouver++;
  }
  return quad;
}

class PointwiseAddSplineManouvers : public SplineManouver
{
protected:
  QuadPoint getNextQuadPoint() override;
  bool isFinished() override;
  Manouvers &_manouvers;

public:
  PointwiseAddSplineManouvers(Manouvers &manouvers) : _manouvers(manouvers) {}
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

public:
  void reset() override {
    _splineManouver.reset();
    _SplineInputFinished = false;
    _t = 0;
    _lastQuad = _splineManouver.getNextQuadPoint();
    _fractionPartOfT = 0;
  }
  RawManouverFromSplineManouver(SplineManouver& splineManouver, float rate) :
      _splineManouver(splineManouver), _rate(rate) {
      reset();
  }
  bool isFinished() override {
    if (!_SplineInputFinished) {
      return false;
    }
    return _fractionPartOfT > 1.0;
  }
  Point getNextPoint() override {
    advanceT();
    return spline(_lastQuad, _fractionPartOfT);
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
  points.push_back(Point(44,
}
