
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectIntegrator : public Integrator
{
public:
    DirectIntegrator(const PropertyList &props)
    {
        // This integrator contains no properties
    }

    /// Compute the radiance value for a given ray. Just return green here
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Intersection its;
        // If the ray has no intersection we can return the black color;
        if (!scene->rayIntersect(ray, its))
            return BLACK;

        Color3f color = BLACK;

        Vector2f sample;

        auto lights = scene->getLights();
        for (Emitter *light : lights)
        {

            EmitterQueryRecord rec;
            rec.ref = its.p;
            Color3f tracedColor = light->sample(rec, sample);

            // Intersection ==> Occlusion ==> Light directly not visible
            if (!scene->rayIntersect(rec.shadowRay))
            {
                Vector3f wi = its.shFrame.toLocal(rec.wi);
                Vector3f d = its.shFrame.toLocal(-ray.d);

                BSDFQueryRecord bRec(wi, d, ESolidAngle);
                bRec.uv = its.uv;
                
                color += its.mesh->getBSDF()->eval(bRec) * Frame::cosTheta(wi) * tracedColor;
            }
        }

        return color;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format("[Direct integrator]");
    }

protected:
    float ray_length;

    const Color3f BLACK = Color3f(0.0);
};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END