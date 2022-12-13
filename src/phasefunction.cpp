#include <nori/phasefunction.h>

NORI_NAMESPACE_BEGIN

float IsotropicPhaseFunction::sample(Vector3f &wo, Vector3f &wi, const Point2f &sample){
    wi = Warp::squareToUniformSphere(sample);
    return INV_FOURPI;
}

std::string IsotropicPhaseFunction::toString() const {
    return tfm::format("[Isotropic phase function ]");
}

NORI_NAMESPACE_END