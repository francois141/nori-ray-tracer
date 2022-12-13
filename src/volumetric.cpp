#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

class VolumetricIntegrator : public Integrator
{
public:
    VolumetricIntegrator(const PropertyList &props)
    {
        // This integrator contains no properties
    }

    /// Compute the radiance value for a given ray. Just return green here
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Medium *medium = scene->getMedium();

        Color3f color = BLACK;
        Color3f attenuation = WHITE;

        Ray3f currentRay = ray;
        float w_mats = 1.0f;

        Intersection its;
        bool intersection = scene->rayIntersect(currentRay,its);

        // Continue until the Russian Roulette says stop
        while (true)
        {
            // Step 1) Find the nearest surface
            float tmax;
            if (intersection) {
                tmax = (its.p - currentRay.o).norm();
            }  else {
                tmax = its.t;
            }
                
            // Step 2) Sample mean free path
            MediumInteractionQuery mQuery;
            mQuery.tMax = tmax;
            Color3f sampledColor = medium->sampler(currentRay, sampler, mQuery);

            // Step 3) Integrate by case distinction
            // First case, we hit the medium
            if (!mQuery.hitObject)
            {
                // Sample the phase function
                Vector3f wo;
                float pdf_mat = medium->getPhaseFunction()->sample(currentRay.d, wo, sampler->next2D());
                

                // Sample an emitter
                const Emitter *light = scene->getRandomEmitter(sampler->next1D());
                EmitterQueryRecord eRec(mQuery.p);

                // Evaluate emitter
                Color3f Li = light->sample(eRec, sampler->next2D()) * scene->getLights().size();
                attenuation *= sampledColor;
                if (!scene->rayIntersect(eRec.shadowRay, its))
                {
                    mQuery.tMax = eRec.shadowRay.maxt;
                    color += attenuation * medium->Tr(mQuery.p, eRec.p) * Li * pdf_mat;
                }

                // Update the russian roulette
                float probability = std::min(attenuation.x(), 0.80f);
                if (sampler->next1D() > probability)
                {
                    return color;
                }
                attenuation /= probability;

                // Continue recursion
                currentRay = Ray3f(mQuery.p,wo.normalized());
                intersection = scene->rayIntersect(currentRay, its);
                if(intersection) {    
                    if (its.mesh->isEmitter()) {
                        EmitterQueryRecord lRec = EmitterQueryRecord(currentRay.o, its.p, its.shFrame.n);
                        float pdf_em = its.mesh->getEmitter()->pdf(lRec);
                        w_mats = pdf_mat + pdf_em > 0.f ? pdf_mat / (pdf_mat + pdf_em) : pdf_mat;
                    }
                }
            }
            // Case 2, we hit an object
            else if(intersection)
            {
                // We add the Le part to the record if the mesh is an emitter
                if (its.mesh->isEmitter())
                {
                    EmitterQueryRecord eRec(currentRay.o, its.p, its.shFrame.n);
                    color += attenuation * w_mats * its.mesh->getEmitter()->eval(eRec) *  medium->Tr(its.p, eRec.p);
                }

                // Sample emitter
                const Emitter *light = scene->getRandomEmitter(sampler->next1D());
                EmitterQueryRecord eRec(its.p);
                Color3f Li = light->sample(eRec, sampler->next2D()) * scene->getLights().size();
                
                // Evaluate emitter
                if (!scene->rayIntersect(eRec.shadowRay))
                {
                    float pdf_em = light->pdf(eRec);
                    float theta = std::max(0.0f, Frame::cosTheta(its.shFrame.toLocal(eRec.wi)));

                    BSDFQueryRecord bRec(its.toLocal(-currentRay.d), its.toLocal(eRec.wi), ESolidAngle);

                    Color3f brdf = its.mesh->getBSDF()->eval(bRec);
                    float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                    float w_ems = (pdf_mat + pdf_em) > 0.0f ? pdf_em / (pdf_mat + pdf_em) : pdf_em;
                    mQuery.tMax = eRec.shadowRay.maxt;
                    color += attenuation * w_ems * brdf * theta * Li * medium->Tr(its.p, eRec.p);
                }

                // Update the russian roulette
                float probability = std::min(attenuation.x(), 0.80f);
                if (sampler->next1D() > probability)
                {
                    return color;
                }
                attenuation /= probability;

                // Sample the BRDF
                BSDFQueryRecord bRec(its.shFrame.toLocal(-currentRay.d));
                Color3f brdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
                attenuation *= brdf;
                float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                // Continue the recursion
                currentRay = Ray3f(its.p, its.toWorld(bRec.wo));
                intersection = scene->rayIntersect(currentRay, its);

                if (intersection) {
                    if (its.mesh->isEmitter()) {
                        EmitterQueryRecord lRec = EmitterQueryRecord(currentRay.o, its.p, its.shFrame.n);
                        float pdf_em = its.mesh->getEmitter()->pdf(lRec);
                        w_mats = pdf_mat + pdf_em > 0.f ? pdf_mat / (pdf_mat + pdf_em) : pdf_mat;
                    }
                    if (bRec.measure == EDiscrete)
                        w_mats = 1.0f;
                }
            }
            // Case 3, we hit tmax without intersections
            else 
            {
                // In this case we can break and return the color
                break;
            }
        }

        return color;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format("[Volumetric path integrator]");
    }

protected:
    float ray_length;

    const Color3f BLACK = Color3f(0.0f);
    const Color3f WHITE = Color3f(1.0f);
};

NORI_REGISTER_CLASS(VolumetricIntegrator, "volumetric");
NORI_NAMESPACE_END