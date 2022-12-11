#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

Medium::Medium(const PropertyList &list) {
    m_absorbtion = list.getColor("sigma_a");
    m_scattering = list.getColor("sigma_s");

    m_extenction = m_absorbtion + m_scattering;
    m_albedo = m_scattering / m_extenction;

    m_phaseFunction = new IsotropicPhaseFunction();
}


Color3f Medium::Tr(const Point3f &source, const Point3f &destination) {
    float norm = (source - destination).norm();

    const float R = exp(-m_extenction.x() * norm);
    const float G = exp(-m_extenction.y() * norm);
    const float B = exp(-m_extenction.z() * norm);

    return Color3f(R,G,B);
}

Color3f Medium::sampler(Ray3f &ray, Sampler *sampler, MediumInteractionQuery &mi) {

    float distance = invTr(sampler->next1D());

    if(distance >= mi.tMax) {
        mi.hitObject = true;
    }
    else {
        mi.p = ray(distance);
        mi.hitObject = false;
        return m_albedo;
    }

    return m_albedo;
}

float Medium::invTr(const float E) {
    return -1.0f * log(1-E) / m_extenction.maxCoeff();
}

PhaseFunction* Medium::getPhaseFunction() {
    return m_phaseFunction;
}

std::string Medium::toString() const {
    return tfm::format("[Homogeneous Medium]");
}

void Medium::addChild(NoriObject *child) {
}

NORI_REGISTER_CLASS(Medium, "medium");
NORI_NAMESPACE_END