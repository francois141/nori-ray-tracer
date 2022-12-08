#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator
{
public:
    PathMatsIntegrator(const PropertyList &props)
    {
        // This integrator contains no properties
    }

    /// Compute the radiance value for a given ray. Just return green here
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Color3f color = BLACK;
        Color3f attenuation = WHITE;

        Ray3f currentRay = ray;

        // Continue until the Russian Roulette says stop
        while (true)
        {

            Intersection its;
            // If the ray has no intersection we can return the black color;
            if (!scene->rayIntersect(currentRay, its))
                return color;

            // We add the Le part to the record if the mesh is an emitter
            if (its.mesh->isEmitter())
            {
                EmitterQueryRecord rec(currentRay.o, its.p, its.shFrame.n);
                color += attenuation * its.mesh->getEmitter()->eval(rec);
            }

            // Update the russian roulette
            float probability = std::min(attenuation.x(),0.99f); 
            if(sampler->next1D() > probability) 
                return color;
            
            attenuation /= probability;

            // Sample the BRDF 
            BSDFQueryRecord bRec(its.shFrame.toLocal(-currentRay.d));
            Color3f brdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            attenuation *= brdf;

            // Continue the recursion
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo));
        }

        return color;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format("[Path Mats integrator]");
    }

protected:
    float ray_length;

    const Color3f BLACK = Color3f(0.0f);
    const Color3f WHITE = Color3f(1.0f);
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END