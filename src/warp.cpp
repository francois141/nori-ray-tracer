/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole)
{
    // Naive implementation using rejection sampling
    Vector3f v;
    do
    {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample)
{
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample)
{
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample)
{
    float angle = 2 * sample.x() * M_PI;
    float size = sqrt(sample.y());
    return Point2f(cos(angle) * size, sin(angle) * size);
}

float Warp::squareToUniformDiskPdf(const Point2f &p)
{
    return p.squaredNorm() <= 1 ? INV_PI : 0.0f;
}

Vector3f Warp::squareToUniformCylinder(const Point2f &sample)
{
    float angle = 2 * sample.x() * M_PI;
    float size = sample.y();
    return Vector3f(cos(angle), sin(angle), 2.0f * size - 1.0f);
}

Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax)
{
    Vector3f cylinder = squareToUniformCylinder(sample);
    cylinder.z() = (abs(cylinder.z() * (1 - cosThetaMax))) + cosThetaMax;
    float r = sqrt(1.0f - pow(cylinder.z(), 2));
    return Vector3f(r * cylinder.x(), r * cylinder.y(), cylinder.z());
}

float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax)
{

    return (v.z() >= cosThetaMax && abs(v.squaredNorm() - 1.0f) < Epsilon) ? 1 / (2 * M_PI * (1 - cosThetaMax)) : 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample)
{
    float theta = acos(1 - 2 * (1 - sample.x()));
    float phi = 2.f * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToUniformSpherePdf(const Vector3f &v)
{
    return abs(v.norm() - 1.0f) < Epsilon ? 0.25f * INV_PI : 0.0f;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample)
{
    float theta = acos(1 - (1 - sample.x()));
    float phi = 2.f * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v)
{
    return (v.z() >= 0 && abs(v.squaredNorm() - 1.0f) < Epsilon) ? 1 / (2.0f * M_PI) : 0.0f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample)
{
    float theta = acos(sqrt(1 - (1 - sample.x())));
    float phi = 2.f * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v)
{
    return (v.z() >= 0 && abs(v.squaredNorm() - 1.0f) < 1.0f) ? INV_PI * v.z() : 0.0f;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha)
{
    float theta = atan(sqrt(-pow(alpha, 2) * log(1 - sample.x())));
    float phi = 2 * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha)
{
    float theta = acos(m.z());
    return (m.z() >= 0 && abs(m.squaredNorm() - 1.0f) < 1.0f) ? exp(-pow(tan(theta), 2) / pow(alpha, 2)) / (M_PI * pow(alpha, 2) * pow(cos(theta), 3)) : 0.0f;
}

Vector3f Warp::squareToUniformTriangle(const Point2f &sample)
{
    float su1 = sqrtf(sample.x());
    float u = 1.f - su1, v = sample.y() * su1;
    return Vector3f(u, v, 1.f - u - v);
}

NORI_NAMESPACE_END
