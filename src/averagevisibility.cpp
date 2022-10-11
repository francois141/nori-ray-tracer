
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AverageVisibilityIntegrator : public Integrator
{
public:
    AverageVisibilityIntegrator(const PropertyList &props)
    {
        ray_length = props.getFloat("length");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return WHITE;

        Normal3f intersectionNormal = its.shFrame.n;
        Ray3f newRay = Ray3f(its.p, Warp::sampleUniformHemisphere(sampler, intersectionNormal), Epsilon, ray_length);

        return scene->rayIntersect(newRay) ? BLACK : WHITE;
    }

    std::string toString() const
    {
        return tfm::format("[Average visibility integrator]");
    }

protected:
    float ray_length;

    const Color3f BLACK = Color3f(0.0);
    const Color3f WHITE = Color3f(1.0);
};

NORI_REGISTER_CLASS(AverageVisibilityIntegrator, "av");
NORI_NAMESPACE_END