#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMatsIntegrator : public Integrator
{
public:
    DirectMatsIntegrator(const PropertyList &props)
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

        // We add the Le part to the record if the mesh is an emitter
        if(its.mesh->isEmitter()) {
            EmitterQueryRecord rec(ray.o, its.p, its.shFrame.n);
            color += its.mesh->getEmitter()->eval(rec);
        }

        // Step 1) Sample the BSDF
        BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
        bRec.uv = its.uv;
        Color3f brdf = its.mesh->getBSDF()->sample(bRec,sampler->next2D());

        // Step 2) Check if we hit a Emitter 
        Ray3f newRay = Ray3f(its.p,its.shFrame.toWorld(bRec.wo));
        Intersection newIntersection;
        if(scene->rayIntersect(newRay,newIntersection) && newIntersection.mesh->isEmitter()) {
            EmitterQueryRecord eRec(its.p,newIntersection.p,newIntersection.shFrame.n);
            color += brdf * newIntersection.mesh->getEmitter()->eval(eRec);
        }

        return color;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format("[Direct Mats integrator]");
    }

protected:
    float ray_length;

    const Color3f BLACK = Color3f(0.0);
};

NORI_REGISTER_CLASS(DirectMatsIntegrator, "direct_mats");
NORI_NAMESPACE_END