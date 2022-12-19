#if !defined(__NORI_PHASE_FUNCTION_H)
#define __NORI_PHASE_FUNCTION_H

#include <nori/object.h>
#include <nori/warp.h>
#include <tinyformat.h>

NORI_NAMESPACE_BEGIN

class PhaseFunction : public NoriObject
{
public:
    PhaseFunction();
    PhaseFunction(const PropertyList &props);

    virtual float sample(Vector3f &wo, Vector3f &wi, const Point2f &sample);

    std::string toString() const override;

    EClassType  getClassType() const override { 
        return EPhaseFunction; 
    }
};


NORI_NAMESPACE_END

#endif