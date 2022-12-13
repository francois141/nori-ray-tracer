#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SpotLight : public Emitter
{
public:
    SpotLight(const PropertyList &props)
    {
        this->position = props.getPoint3("position");
        this->power = props.getColor("color");
        this->direction = props.getVector3("direction").normalized();
        
        this->cosFalloffStart = std::cos(M_PI / 180 * props.getFloat("falloffStart"));
        this->cosTotalWidth = std::cos(M_PI / 180 * props.getFloat("totalWidth"));
    }

    Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const
    {
        lRec.wi = (this->position - lRec.ref).normalized();
        lRec.p = this->position;
        lRec.pdf = 1.0f;
        lRec.n = this->direction;
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, (this->position - lRec.ref).norm() - Epsilon);

        return this->power * falloff(-lRec.wi) / (4.f * M_PI * (lRec.ref - lRec.p).squaredNorm());
    }

    float falloff(const Vector3f &w) const {
        float cosTheta = this->direction.dot(w.normalized());
        if(cosTheta < cosTotalWidth) return 0;
        if(cosTheta > cosFalloffStart) return 1;
        // Linear interpolate between cosFallOffStart & cosTotalWidth
        return (std::acos(cosTotalWidth) - std::acos(cosTheta))/ (std::acos(cosTotalWidth) - std::acos(cosFalloffStart));
    }

    Color3f eval(const EmitterQueryRecord &lRec) const
    {
        Color3f c = this->power / (4.f * M_PI);
        return c * 2 * M_PI * (1 - 0.5 * (cosFalloffStart + cosTotalWidth));
    }

    float pdf(const EmitterQueryRecord &lRec) const
    {
        return lRec.pdf;
    }

    std::string toString() const
    {
        return tfm::format("[Spot light emitter \n"
            "position = %s  \n"
            "power = %s \n"
            "direction = %s \n"
            "cosFalloffStart = %f \n"
            "cosTotalWidth = %f \n"
        "]",
        this->position.toString(),this->power.toString(),this->direction.toString(),this->cosFalloffStart,this->cosTotalWidth);
    }

protected:
    std::string m_myProperty;

    Point3f position;
    Vector3f direction;
    Color3f power;

    float cosFalloffStart;
    float cosTotalWidth;
};

NORI_REGISTER_CLASS(SpotLight, "spotlight");
NORI_NAMESPACE_END