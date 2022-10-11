/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Sphere : public Shape
{
public:
    Sphere(const PropertyList &propList)
    {
        m_position = propList.getPoint3("center", Point3f());
        m_radius = propList.getFloat("radius", 1.f);

        m_bbox.expandBy(m_position - Vector3f(m_radius));
        m_bbox.expandBy(m_position + Vector3f(m_radius));
    }

    virtual BoundingBox3f getBoundingBox(uint32_t index) const override { return m_bbox; }

    virtual Point3f getCentroid(uint32_t index) const override { return m_position; }

    // Ressource used as inspiration : https://raytracing.github.io/books/RayTracingInOneWeekend.html
    virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const override
    {
        Vector3f oc = ray.o - m_position;

        float a = ray.d.dot(ray.d);
        float b = 2.0 * oc.dot(ray.d);
        float c = oc.dot(oc) - m_radius*m_radius;

        float discriminant = (b * b - 4 * a * c);

        bool intersect = discriminant > 0;

        if (!intersect)
            return false;

        float delta = sqrt(b * b - 4 * a * c);

        float t1 = (-b - delta) / (2 * a);
        float t2 = (-b + delta) / (2 * a);

        if (ray.mint <= t1 && t1 <= ray.maxt)
        {
            t = t1;
            return true;
        }

        if (ray.mint <= t2 && t2 <= ray.maxt)
        {
            t = t2;
            return true;
        }

        return false;
    }

    virtual void setHitInformation(uint32_t index, const Ray3f &ray, Intersection &its) const override
    {
        Point3f intersectionPoint = ray.o + its.t * ray.d;
        its.p = intersectionPoint;

        Vector3f n = (intersectionPoint - m_position).normalized();
        
        Frame frame = Frame(n);
        its.shFrame = frame;
        its.geoFrame = frame;

        Point2f coords = sphericalCoordinates(n);
        coords[0] = 0.5 + coords[0] / (2*M_PI);
        coords[1] /= M_PI;
        its.uv = coords;
    }

    virtual void sampleSurface(ShapeQueryRecord &sRec, const Point2f &sample) const override
    {
        Vector3f q = Warp::squareToUniformSphere(sample);
        sRec.p = m_position + m_radius * q;
        sRec.n = q;
        sRec.pdf = std::pow(1.f / m_radius, 2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f, 0.0f, 1.0f));
    }
    virtual float pdfSurface(const ShapeQueryRecord &sRec) const override
    {
        return std::pow(1.f / m_radius, 2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f, 0.0f, 1.0f));
    }

    virtual std::string toString() const override
    {
        return tfm::format(
            "Sphere[\n"
            "  center = %s,\n"
            "  radius = %f,\n"
            "  bsdf = %s,\n"
            "  emitter = %s\n"
            "]",
            m_position.toString(),
            m_radius,
            m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
            m_emitter ? indent(m_emitter->toString()) : std::string("null"));
    }

protected:
    Point3f m_position;
    float m_radius;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
