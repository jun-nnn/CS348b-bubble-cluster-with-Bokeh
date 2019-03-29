
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
#include "shapes/distanceestimator.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"
#include "geometry.h"

namespace pbrt {

// Sphere Method Definitions

bool DistanceEstimator::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const {
  Float t = 0.0;
  Float d = 0.0;

  Vector3f oErr, dErr;
  Ray ray = (*WorldToObject)(r, &oErr, &dErr);

  Point3f pHit;
  Vector3f dpdu, dpdv;
  int i = 0;
  //Float d = Evaluate(ray.o);

  while (t < ray.tMax && i <= maxIters) {
    i++;
    d = Evaluate(ray(t));
    // update tHit
    if (tHit != nullptr){
       *tHit = (Float)t;
    }
    // if intersect
    if (d < hitEpsilon && d>0) {
      pHit = ray(t);
      Vector3f pError = Abs(Vector3f(rayEpsilonMultiplier*hitEpsilon,
				                             rayEpsilonMultiplier*hitEpsilon,
				                             rayEpsilonMultiplier*hitEpsilon));

      Vector3f pHitNormal = CalculateNormal(pHit, normalEpsilon, -ray.d);

      CoordinateSystem(pHitNormal, &dpdu, &dpdv);

      if (isect != nullptr){
	        *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, pError, Point2f(0,0), -ray.d, dpdu, dpdv, Normal3f(0,0,0), Normal3f(0,0,0), ray.time, this));
      }
      return true;
    }
    t = t + (d/(ray.d).Length());
  }
  return false;
}

Interaction DistanceEstimator::Sample(const Point2f &u, Float *pdf) const {
    LOG(FATAL) << "DE::Sample not implemented.";
    return Interaction();
}

Vector3f DistanceEstimator::CalculateNormal(const Point3f& pos, float eps,
       const Vector3f& defaultNormal) const {
 const Vector3f v1 = Vector3f( 1.0,-1.0,-1.0);
 const Vector3f v2 = Vector3f(-1.0,-1.0, 1.0);
 const Vector3f v3 = Vector3f(-1.0, 1.0,-1.0);
 const Vector3f v4 = Vector3f( 1.0, 1.0, 1.0);

 const Vector3f normal = v1 * Evaluate( pos + v1*eps ) +
             v2 * Evaluate( pos + v2*eps ) +
             v3 * Evaluate( pos + v3*eps ) +
             v4 * Evaluate( pos + v4*eps );
 const Float length = normal.Length();

 return length > 0 ? (normal/length) : defaultNormal;
}
}  // namespace pbrt
