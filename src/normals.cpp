#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class NormalIntegrator : public Integrator
{
    public:
        NormalIntegrator(const PropertyList &props)
        {

        }

        /// Compute the radiance value for a given ray. Just return green here
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
        {
            Intersection its;
            if(!scene->rayIntersect(ray,its))
                return Color3f(0.0f);

            Normal3f n = its.shFrame.n.cwiseAbs();
            return Color3f(n.x(), n.y(), n.z());
        }

        /// Return a human-readable description for debugging purposes
        std::string toString() const
        {
            return tfm::format("[Normal integrator]");
        }

    protected:
        std::string m_myProperty;
};

NORI_REGISTER_CLASS(NormalIntegrator, "normals");
NORI_NAMESPACE_END