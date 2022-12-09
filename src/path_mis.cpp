#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator
{
public:
    PathMisIntegrator(const PropertyList &props)
    {
        // This integrator contains no properties
    }

    /// Compute the radiance value for a given ray. Just return green here
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Color3f color = BLACK;
        Color3f attenuation = WHITE;

        Ray3f currentRay = ray;

        float w_mats = 1.0f;

        Intersection its;
        // If the ray has no intersection we can return the black color;
        if (!scene->rayIntersect(currentRay, its))
            return color;

        // Continue until the Russian Roulette says stop
        while (true)
        {
            // We add the Le part to the record if the mesh is an emitter
            if (its.mesh->isEmitter())
            {
                EmitterQueryRecord eRec(currentRay.o, its.p, its.shFrame.n);
                color += attenuation * w_mats * its.mesh->getEmitter()->eval(eRec);
            }

            // Sample EMS
            const Emitter *light = scene->getRandomEmitter(sampler->next1D());
            EmitterQueryRecord eRec(its.p);
            Color3f Li = light->sample(eRec, sampler->next2D()) * scene->getLights().size();

            float pdf_em = light->pdf(eRec);

            if (!scene->rayIntersect(eRec.shadowRay))
            {
                float theta = std::max(0.0f, Frame::cosTheta(its.shFrame.toLocal(eRec.wi)));

                BSDFQueryRecord bRec(its.toLocal(-currentRay.d), its.toLocal(eRec.wi), ESolidAngle);
                bRec.uv = its.uv;
                Color3f brdf = its.mesh->getBSDF()->eval(bRec);
                float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                float w_ems = (pdf_mat + pdf_em) > 0.0f ? pdf_em / (pdf_mat + pdf_em) : pdf_em;

                color += attenuation * w_ems * brdf * theta * Li;
            }

            // Update the russian roulette
            float probability = std::min(attenuation.x(), 0.99f);
            if (sampler->next1D() > probability)
            {
                return color;
            }
            attenuation /= probability;

            // Sample the BRDF
            BSDFQueryRecord bRec(its.shFrame.toLocal(-currentRay.d));
            bRec.uv = its.uv;
            Color3f brdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            attenuation *= brdf;

            // Continue the recursion
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo));

            // Sample MATS
            float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

            Point3f origin = its.p;
            if (!scene->rayIntersect(currentRay, its))
                return color;

            if (its.mesh->isEmitter())
            {
                EmitterQueryRecord eRec = EmitterQueryRecord(origin, its.p, its.shFrame.n);
                float pdf_em = its.mesh->getEmitter()->pdf(eRec);
                w_mats = pdf_mat + pdf_em > 0.f ? pdf_mat / (pdf_mat + pdf_em) : pdf_mat;
            }

            if (bRec.measure == EDiscrete)
            {
                w_mats = 1.0f;
            }
        }

        return color;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format("[Path Mis integrator]");
    }

protected:
    float ray_length;

    const Color3f BLACK = Color3f(0.0f);
    const Color3f WHITE = Color3f(1.0f);
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END