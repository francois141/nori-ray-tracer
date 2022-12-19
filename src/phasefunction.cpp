#include <nori/phasefunction.h>

NORI_NAMESPACE_BEGIN

PhaseFunction::PhaseFunction() {
    return;
}

PhaseFunction::PhaseFunction(const PropertyList &props) {
    return;
}

float PhaseFunction::sample(Vector3f &wo, Vector3f &wi, const Point2f &sample){
    wi = Warp::squareToUniformSphere(sample);
    return INV_FOURPI;
}

std::string PhaseFunction::toString() const {
    return tfm::format("[Isotropic phase function ]");
}

NORI_REGISTER_CLASS(PhaseFunction, "isotropic");
NORI_NAMESPACE_END