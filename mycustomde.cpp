
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */
#include <math.h>
#include "shape.h"
#include "shapes/mycustomde.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"
#include "geometry.h"

namespace pbrt {

// Sphere Method Definitions
Bounds3f MyCustomDE::ObjectBound() const {
    //return Bounds3f(Point3f(-10000, -10000, -10000),
    //                Point3f(10000, 10000, 10000));
    return Bounds3f(2*Point3f(-radius, -radius, -radius),
                    2*Point3f(radius, radius, radius));
}

Float MyCustomDE::Area() const { return 2*M_PI * radius * (radius - (-radius));}

Float MyCustomDE::Evaluate(const Point3f& p) const{
  // original ice cream shape
  /*Point2f c = Point2f(0.95, -(Float)sqrt(1-0.95*0.95));
  Float q = (Float) sqrt(p.x*p.x + p.z*p.z);
  Float cone = (c.x*q + c.y*(p.y+1));
  Float sphere = (Float) sqrt (p.x*p.x + (p.y-0.5)*(p.y-0.5) + p.z*p.z) - radius;
  if (p.y > 0.2 || sphere < cone) {
    return sphere;
  } else {
    return cone;
  }*/
  Point3f A = Point3f(0,0,0.1);
  if (radius > 2){
    A = Point3f(-1,-1,1.5);
  }
  //Point3f A = Point3f(-0.1, 0.2, -0.1);
  Point3f B;
  Point3f C;
  Point3f D;
  Point3f E;
  Point3f F;
  Point3f G;
  Float ra = 1;
  //Float ra = 0.8;
  Float rb = 0.7;
  if (radius > 2){
    rb = 0.5;
  }
  Float rc = 0.6;
  Float rg = 0.5;
  Float rd = (ra*rb)/(ra-rb);
  Float re = (ra*rc)/(ra-rc);
  Float rf = (rb*rc)/(rb-rc);
  Float AB = (Float) sqrt(ra*ra + rb*rb -ra*rb);
  Float AD = (Float) sqrt(ra*ra + rd*rd -ra*rd);
  Float AC = (Float) sqrt(ra*ra + rc*rc -ra*rc);
  Float AE = (Float) sqrt(ra*ra + re*re -ra*re);
  Float BC = (Float) sqrt(rb*rb + rc*rc -rb*rc);
  Float CG = (Float) sqrt(rc*rc + rg*rg -rc*rg);
  Float BG = (Float) sqrt(rb*rb + rg*rg -rb*rg);
  B = A + Point3f(0, AB, 0);
  Float Cx = (AC*AC+AB*AB-BC*BC)/(2*AB);
  Float Cy = (Float) sqrt(AC*AC - Cx*Cx);
  C = A + Point3f(0, Cx, Cy);
  G = A + Point3f(Cx, 0, Cy);
  Float distA = (Float) sqrt ((p.x-A.x)*(p.x-A.x) + (p.y-A.y)*(p.y-A.y) + (p.z-A.z)*(p.z-A.z)) - ra;
  Float distB = (Float) sqrt ((p.x-B.x)*(p.x-B.x) + (p.y-B.y)*(p.y-B.y) + (p.z-B.z)*(p.z-B.z)) - rb;
  Float distC = (Float) sqrt ((p.x-C.x)*(p.x-C.x) + (p.y-C.y)*(p.y-C.y) + (p.z-C.z)*(p.z-C.z)) - rc;
  Float distG = (Float) sqrt ((p.x-G.x)*(p.x-G.x) + (p.y-G.y)*(p.y-G.y) + (p.z-G.z)*(p.z-G.z)) - rc;
  if (radius >=4){
    return std::min(distA, distB)+(1-(radius-4));
  }
  if (radius >=2){
    return std::min(distA, distB)+(1-(radius-2));
  }
  return std::min(distA, std::min(distB, distC))+(1-radius);
}


std::shared_ptr<Shape> CreateMyCustomDEShape(const Transform *o2w,
                                         const Transform *w2o,
                                         bool reverseOrientation,
                                         const ParamSet &params) {
    Float radius = params.FindOneFloat("radius", 2.f);
    int maxIters = params.FindOneFloat("maxIters", 1000);
    Float hitEpsilon = params.FindOneFloat("hitEpsilon", 0.0001);
    Float rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 10);
    Float normalEpsilon = params.FindOneFloat("normalEpsilon", 0.0001);
    return std::make_shared<MyCustomDE>(o2w, w2o, reverseOrientation,
      radius, maxIters, hitEpsilon, rayEpsilonMultiplier, normalEpsilon);
}

}  // namespace pbrt
