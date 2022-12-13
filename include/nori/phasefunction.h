#include <nori/object.h>
#include <nori/warp.h>
#include <tinyformat.h>

#if !defined(__NORI_PHASE_FUNCTION_H)
#define __NORI_PHASE_FUNCTION_H

#define PURE_VIRTUAL 0

NORI_NAMESPACE_BEGIN

class PhaseFunction : public NoriObject
{
public:
    virtual float sample(Vector3f &wo, Vector3f &wi, const Point2f &sample) = PURE_VIRTUAL;
};

class IsotropicPhaseFunction  : public PhaseFunction
{
    public:
    float sample(Vector3f &wo, Vector3f &wi, const Point2f &sample);
    std::string toString() const;

    EClassType getClassType() const override {
        return EPhaseFunction;
    }
};

NORI_NAMESPACE_END

#endif