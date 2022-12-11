#include <nori/object.h>
#include <nori/sampler.h>
#include <nori/phasefunction.h>

#if !defined(__NORI_MEDIUM_H)
#define __NORI_MEDIUM_H

NORI_NAMESPACE_BEGIN

struct MediumInteractionQuery {
    Point3f p; 
    float tMax;
    bool hitObject;
};

class Medium : public NoriObject {

    public:
    Medium(const PropertyList &list);

    Color3f Tr(const Point3f &source, const Point3f &destination);
    Color3f sampler(Ray3f &ray, Sampler *sampler, MediumInteractionQuery &mi);

    float invTr(const float E);
    PhaseFunction* getPhaseFunction();

    void addChild(NoriObject *child) override;
    std::string toString() const override;
    EClassType getClassType() const override { 
        return EMedium; 
    }

    private:

    // Given coefficients
    Color3f m_absorbtion;
    Color3f m_scattering;
    PhaseFunction *m_phaseFunction;

    // Derived coefficients
    Color3f m_extenction;
    Color3f m_albedo;

};

NORI_NAMESPACE_END

#endif