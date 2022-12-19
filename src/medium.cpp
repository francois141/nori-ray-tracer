#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

Medium::Medium(const PropertyList &list) {

    // Base components
    m_absorbtion = list.getColor("sigma_a");
    m_scattering = list.getColor("sigma_s");

    // Derived parameterse
    m_extenction = m_absorbtion + m_scattering;
    m_albedo = m_scattering / m_extenction;

    // Add bounding box
    Vector3f size_bbox = list.getVector3("box_size").cwiseAbs();
    Vector3f origin_bbox = list.getVector3("box_origin");
    bounds = BoundingBox3f(origin_bbox - size_bbox, origin_bbox + size_bbox);
}


Color3f Medium::Tr(const Point3f &source, const Point3f &destination) {

    // Check for intersection
    float nearT,farT;
    Ray3f ray = Ray3f(source,(destination-source).normalized());
    if(!bounds.rayIntersect(ray,nearT,farT)) {
        return Color3f(1.0f);
    }

    // Compute the starting point
    Point3f startPoint = Point3f(0.0f);
    if(bounds.contains(source)) {
        startPoint = source;
    } else {
        startPoint = source + ray.d.normalized() * nearT;
    }

    // Compute the end point
    Point3f endPoint = Point3f(0.0f);
    if(bounds.contains(destination)) {
        endPoint = destination;
    } else {
        endPoint = source + ray.d.normalized() * farT;
    }

    // Distance between the two points
    float norm = (endPoint - startPoint).norm();

    // Compute attenuation
    const float R = exp(-m_extenction.x() * norm);
    const float G = exp(-m_extenction.y() * norm);
    const float B = exp(-m_extenction.z() * norm);

    // Return the color
    return Color3f(R,G,B);
}

Color3f Medium::sampler(Ray3f &ray, Sampler *sampler, MediumInteractionQuery &mi) {

    // No intersection at all
    float nearT,farT;
    if(!bounds.rayIntersect(ray,nearT,farT)) {
        mi.hitObject = true;
        return Color3f(1.0f);
    }

    // There is an intesection
    Point3f startPoint = Point3f(0.0f);
    if(bounds.contains(ray.o)) {
        startPoint = ray.o;
    } else {
        startPoint = ray.o + ray.d.normalized() * nearT;
    }

    // First we get the distance
    float distance = (startPoint - ray.o).norm() + invTr(sampler->next1D());

    // Then we check if there is an intersection or not
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
    switch (child->getClassType()) {
        case EPhaseFunction:
            if (m_phaseFunction) {
                throw NoriException("Already registered");
            }
            m_phaseFunction = static_cast<PhaseFunction*>(child);
            break;
        default:
            throw NoriException("Can only register a phase function");
    }
}

NORI_REGISTER_CLASS(Medium, "medium");
NORI_NAMESPACE_END