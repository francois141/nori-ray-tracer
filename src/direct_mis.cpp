#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMisIntegrator : public Integrator
{
public:
    DirectMisIntegrator(const PropertyList &props)
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
        if (its.mesh->isEmitter())
        {
            EmitterQueryRecord rec(ray.o, its.p, its.shFrame.n);
            color += its.mesh->getEmitter()->eval(rec);
        }

        // Add the direct integrator part
        auto lights = scene->getLights();
        for (Emitter *light : lights)
        {
            EmitterQueryRecord rec(its.p);
            Color3f tracedColor = light->sample(rec, sampler->next2D());

            float pdf_em = light->pdf(rec);
            
            // Intersection ==> Occlusion ==> Light directly not visible
            if (!scene->rayIntersect(rec.shadowRay))
            {
                Vector3f wi = its.shFrame.toLocal(rec.wi);
                Vector3f d = its.shFrame.toLocal(-ray.d);

                float cosTheta = Frame::cosTheta(wi);

                BSDFQueryRecord bRec(d, wi, ESolidAngle);
                bRec.uv = its.uv;

                Color3f new_color = its.mesh->getBSDF()->eval(bRec);

                float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                float w_em = pdf_mat + pdf_em > 0.f ? pdf_em / (pdf_mat + pdf_em) : pdf_em;

                color += w_em * new_color * tracedColor * cosTheta;
            }
        }

        // Step 1) Sample the BSDF
        BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
        bRec.uv = its.uv;
        Color3f sensibility = its.mesh->getBSDF()->sample(bRec,sampler->next2D());
        float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

        // Step 2) Check if we hit a Emitter
        Ray3f newRay = Ray3f(its.p,its.shFrame.toWorld(bRec.wo));
        Intersection newIntersection;
        
        if(scene->rayIntersect(newRay,newIntersection)) {
            if(newIntersection.mesh->isEmitter()) {
                EmitterQueryRecord eRec(its.p,newIntersection.p,newIntersection.shFrame.n);
                Color3f emmitedColor = newIntersection.mesh->getEmitter()->eval(eRec);

                float pdf_em = newIntersection.mesh->getEmitter()->pdf(eRec);

                float w_mat = pdf_mat + pdf_em > 0.f ? pdf_mat / (pdf_mat + pdf_em) : 0.0f;

                color += w_mat * sensibility * emmitedColor;
            }
        }

        return color;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format("[Direct MIS integrator]");
    }

protected:
    float ray_length;

    const Color3f BLACK = Color3f(0.0);
};

NORI_REGISTER_CLASS(DirectMisIntegrator, "direct_mis");
NORI_NAMESPACE_END