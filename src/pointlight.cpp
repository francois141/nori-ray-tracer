#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class PointLight : public Emitter
{
public:
    PointLight(const PropertyList &props)
    {
        this->position = props.getPoint3("position", Point3f());
        this->power = props.getColor("power", Color3f());
    }

    Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const
    {
        lRec.wi = (this->position - lRec.ref).normalized();
        lRec.p = this->position;
        lRec.pdf = PDF_VALUE;
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, (this->position - lRec.ref).norm() - Epsilon);

        return this->power / (4.f * M_PI * (this->position - lRec.ref).squaredNorm());
    }

    Color3f eval(const EmitterQueryRecord &lRec) const
    {
        return this->power / (4.f * M_PI * (this->position - lRec.ref).squaredNorm());
    }

    float pdf(const EmitterQueryRecord &lRec) const
    {
        return PDF_VALUE;
    }

    std::string toString() const
    {
        return tfm::format("[Point light emitter position = %s power = %s]",
                           this->position.toString(),
                           this->power.toString());
    }

protected:
    std::string m_myProperty;

    Point3f position;
    Color3f power;

    const float PDF_VALUE = 1.0f;
};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END