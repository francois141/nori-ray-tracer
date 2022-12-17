#include <nori/object.h>
#include <nori/texture.h>
#include <nori/sampler.h>
#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include "math.h"

#define N_OCTAVES 9

NORI_NAMESPACE_BEGIN

class PerlinSphere : public Shape {
public:
    PerlinSphere(const PropertyList &propList) {
        m_position = propList.getPoint3("center", Point3f());
        m_radius = propList.getFloat("radius", 1.f);
        m_height = propList.getFloat("noiseHeight", 1.0f);

        m_bbox.expandBy(m_position - Vector3f(m_radius));
        m_bbox.expandBy(m_position + Vector3f(m_radius));
    }

    virtual BoundingBox3f getBoundingBox(uint32_t index) const override { return m_bbox; }

    virtual Point3f getCentroid(uint32_t index) const override { return m_position; }

    virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const override {
        Vector3f oc = ray.o - m_position;

        float a = ray.d.dot(ray.d);
        float b = 2.0 * oc.dot(ray.d);
        float c = oc.dot(oc) - m_radius * m_radius;
        float t0, t1;

        // Solve implicit equation
        size_t n_sols = solve_quadratic(a, b, c, &t0, &t1);

        if(n_sols == 0) {
            return false;
        }

        if(n_sols == 1) {
            t = t0;
            // Return whether the obtained solution is valid or not
            if (ray.mint <= t && t < ray.maxt) {
                return perlinRayIntersect(a, b, ray, t);
            }
            return false;
        }

        // We found 2 solutions
        t = std::min(t0, t1);
        if(ray.mint <= t && t < ray.maxt) {
            return perlinRayIntersect(a, b, ray, t);
        }
        // First t wasn't valid, let's try the second one
        t = std::max(t0, t1);
        if(ray.mint <= t && t < ray.maxt) {
            return perlinRayIntersect(a, b, ray, t);
        }
        return false;
    }

    virtual void setHitInformation(uint32_t index, const Ray3f &ray, Intersection &its) const override {
        Point3f intersectionPoint = ray(its.t);
        its.p = intersectionPoint;

        Vector3f n = (intersectionPoint - m_position).normalized();
        
        Frame frame = Frame(n);
        its.shFrame = frame;
        its.geoFrame = frame;

        Point2f coords = sphericalCoordinates(n);
        coords[0] = 0.5 + coords[0] * (INV_TWOPI);
        coords[1] *= INV_PI;
        its.uv = coords;
    }

    virtual void sampleSurface(ShapeQueryRecord &sRec, const Point2f &sample) const override {
        Vector3f q = Warp::squareToUniformSphere(sample);
        sRec.p = m_position + m_radius * q;

        // Compute noised radius
        float r = getNoisedRadius(sRec.p);
        sRec.p = m_position + r * q;
        sRec.n = q;
        sRec.pdf = std::pow(1.f / r, 2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f, 0.0f, 1.0f));
    }

    virtual float pdfSurface(const ShapeQueryRecord &sRec) const override {
        float r = getNoisedRadius(sRec.p);
        return std::pow(1.f / r, 2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f, 0.0f, 1.0f));
    }

    virtual std::string toString() const override {
        return tfm::format(
            "PerlinSphere[\n"
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

private:
    // Given an intersection point, update the radius and check if ray still intersects
    bool perlinRayIntersect(const float a, const float b, const Ray3f &ray, float &t) const {
        Point2f its_p = ray(t);
        Vector3f oc = ray.o - m_position;
        float r = getNoisedRadius(its_p); // Apply the noise to the radius

        // Recompute c using the new radius
        float c = oc.dot(oc) - r * r;
        float t0, t1;

        // Solve implicit equation with new radius
        size_t n_sols = solve_quadratic(a, b, c, &t0, &t1);

        // Same as with the sphere
        if(n_sols == 0) {
            return false;
        }

        if(n_sols == 1) {
            t = t0;
            // Return whether the obtained solution is valid or not
            return (ray.mint <= t && t < ray.maxt);
        }

        // We found 2 solutions
        t = std::min(t0, t1);
        if(ray.mint <= t && t < ray.maxt) {
            return true;
        }
        // First t wasn't valid, let's try the second one
        t = std::max(t0, t1);
        if(ray.mint <= t && t < ray.maxt) {
            return true;
        }
        return false;
    }

    // The idea is we want a sphere with a variable radius
    // So we recompute the radius depending on the real intersection point
    float getNoisedRadius(const Point2f &its_p) const {
        float r_scale = computeRadiusNoise(its_p);
        return m_radius * r_scale;
    }

    // Based off of the slides from my Intro to CG course at EPFL
    float computeRadiusNoise(const Point2f &sample) const {
        float res = 0.0f;
        float freq = 1.0f / m_height;
        float amp = 1.0f;
        float lac = 2.0f;

        // Scale the noise up exponentially
        for (int i = 0; i < N_OCTAVES; ++i) {
            res += interpolatedNoise(sample.x() * freq, sample.y() * freq) * amp;
            freq = pow(2, i);
            amp = pow(lac, static_cast<float>(i));
        }

        return (res / 255);
    }

    float interpolatedNoise(float x, float y) const {
        int x_ = static_cast<int>(x);
        int y_ = static_cast<int>(y);

        float dx = x - x_;
        float dy = y - y_;

        float v0 = billinearNoise(x_, y_);
        float v1 = billinearNoise(x_ + 1, y_);
        float v2 = billinearNoise(x_, y_ + 1);
        float v3 = billinearNoise(x_ + 1, y_ + 1);

        float i0 = cosineInterpolation(v0 , v1 , dx);
        float i1 = cosineInterpolation(v2 , v3 , dx);

        return cosineInterpolation(i0 , i1 , dy);
    }

    // Cosine-based interpolation of two points
    float cosineInterpolation(float a, float b, float x) const {
        double ft = x * M_PI;
        double f = (1 - cos(ft)) * 0.5;
        return  a * (1 - f) + b * f;
    }

    // Interpolation between noise all neighboring pixels
    float billinearNoise(int x, int y) const {
        return ((noise(x - 1, y) + noise(x + 1, y) + noise(x, y - 1) + noise(x, y + 1)) / 8.0f) + // Side neighbors
        ((noise(x - 1, y - 1) + noise(x + 1, y - 1) + noise(x - 1, y + 1) + noise(x + 1, y + 1)) / 16.0f) + // Corner neighbors 
        (noise(x, y) / 4.0f); // Current pixel
    }

    // 2-Point gradient noise function, based off of ICG slides
    float noise(int x, int y) const {
        int n = x + y * 57;
        n = (n << 13) ^ n;
        return (1.0f - ((n * ((n * n * 15731) + 789221) +  1376312589) & 0x7fffffff) / 1073741824.0);
    }

protected:
    Point3f m_position;
    float m_radius;
    float m_height; // For Perlin Noise
};

NORI_REGISTER_CLASS(PerlinSphere, "perlinsphere");
NORI_NAMESPACE_END