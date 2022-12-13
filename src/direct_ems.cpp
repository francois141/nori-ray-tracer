#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectEmsIntegrator : public Integrator
{
public:
    DirectEmsIntegrator(const PropertyList &props)
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

        // If the material is an Emitter ==> Add the emmission part to the result
        if(its.mesh->isEmitter()) {
            EmitterQueryRecord rec(ray.o, its.p, its.shFrame.n);
            color += its.mesh->getEmitter()->eval(rec);
        }

        // Add the direct integrator part
        auto lights = scene->getLights();
        for (Emitter *light : lights)
        {
            EmitterQueryRecord rec;
            rec.ref = its.p;
            Color3f tracedColor = light->sample(rec, sampler->next2D());

            // Intersection ==> Occlusion ==> Light directly not visible
            if (!scene->rayIntersect(rec.shadowRay))
            {
                Vector3f wi = its.shFrame.toLocal(rec.wi);
                Vector3f d = its.shFrame.toLocal(-ray.d);

                BSDFQueryRecord bRec(d, wi, ESolidAngle);
                bRec.uv = its.uv;

                color += its.mesh->getBSDF()->eval(bRec) * Frame::cosTheta(wi) * tracedColor;
            }
        }

        return color;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format("[Direct EMS integrator]");
    }

protected:
    float ray_length;

    const Color3f BLACK = Color3f(0.0);
};

NORI_REGISTER_CLASS(DirectEmsIntegrator, "direct_ems");
NORI_NAMESPACE_END